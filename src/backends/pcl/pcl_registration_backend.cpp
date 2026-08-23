#include "pcl_registration_backend.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_ad::backends::pcl_backend {
namespace {

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Minimal double-precision rigid-motion primitives. Transforms are 4x4 row-major
// matrices, consistent with the public RigidTransform storage layout.
// ---------------------------------------------------------------------------

struct V3 final {
  double x{};
  double y{};
  double z{};
};

[[nodiscard]] V3 operator+(V3 left, V3 right) noexcept {
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}
[[nodiscard]] V3 operator-(V3 left, V3 right) noexcept {
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}
[[nodiscard]] V3 operator*(double scalar, V3 vector) noexcept {
  return {scalar * vector.x, scalar * vector.y, scalar * vector.z};
}
[[nodiscard]] double dot(V3 left, V3 right) noexcept {
  return left.x * right.x + left.y * right.y + left.z * right.z;
}
[[nodiscard]] V3 cross(V3 left, V3 right) noexcept {
  return {left.y * right.z - left.z * right.y, left.z * right.x - left.x * right.z,
          left.x * right.y - left.y * right.x};
}
[[nodiscard]] double norm(V3 vector) noexcept {
  return std::sqrt(dot(vector, vector));
}
[[nodiscard]] V3 normalized(V3 vector) noexcept {
  const double length = norm(vector);
  return length > 0.0 ? (1.0 / length) * vector : V3{};
}

using Mat4 = std::array<double, 16>;

[[nodiscard]] V3 apply_point(const Mat4& transform, V3 point) noexcept {
  return {transform[0] * point.x + transform[1] * point.y + transform[2] * point.z + transform[3],
          transform[4] * point.x + transform[5] * point.y + transform[6] * point.z + transform[7],
          transform[8] * point.x + transform[9] * point.y + transform[10] * point.z +
              transform[11]};
}

[[nodiscard]] V3 apply_direction(const Mat4& transform, V3 direction) noexcept {
  return {transform[0] * direction.x + transform[1] * direction.y + transform[2] * direction.z,
          transform[4] * direction.x + transform[5] * direction.y + transform[6] * direction.z,
          transform[8] * direction.x + transform[9] * direction.y + transform[10] * direction.z};
}

[[nodiscard]] Mat4 multiply(const Mat4& left, const Mat4& right) noexcept {
  Mat4 result{};
  for (std::size_t row = 0; row < 4U; ++row) {
    for (std::size_t column = 0; column < 4U; ++column) {
      double sum = 0.0;
      for (std::size_t index = 0; index < 4U; ++index) {
        sum += left[row * 4U + index] * right[index * 4U + column];
      }
      result[row * 4U + column] = sum;
    }
  }
  return result;
}

[[nodiscard]] Mat4 inverse_rigid(const Mat4& transform) noexcept {
  const double r00 = transform[0], r01 = transform[1], r02 = transform[2];
  const double r10 = transform[4], r11 = transform[5], r12 = transform[6];
  const double r20 = transform[8], r21 = transform[9], r22 = transform[10];
  const double tx = transform[3], ty = transform[7], tz = transform[11];
  Mat4 inverse{};
  inverse[0] = r00;
  inverse[1] = r10;
  inverse[2] = r20;
  inverse[4] = r01;
  inverse[5] = r11;
  inverse[6] = r21;
  inverse[8] = r02;
  inverse[9] = r12;
  inverse[10] = r22;
  inverse[3] = -(r00 * tx + r10 * ty + r20 * tz);
  inverse[7] = -(r01 * tx + r11 * ty + r21 * tz);
  inverse[11] = -(r02 * tx + r12 * ty + r22 * tz);
  inverse[15] = 1.0;
  return inverse;
}

// Builds the incremental transform from a twist (rotation vector + translation) via Rodrigues.
[[nodiscard]] Mat4 delta_from_twist(V3 rotation, V3 translation) noexcept {
  Mat4 delta{1.0, 0.0, 0.0, translation.x, 0.0, 1.0, 0.0, translation.y,
             0.0, 0.0, 1.0, translation.z, 0.0, 0.0, 0.0, 1.0};
  const double angle = norm(rotation);
  if (angle < 1.0e-12) {
    // First-order approximation: R ~= I + [rotation]_x.
    delta[1] = -rotation.z;
    delta[2] = rotation.y;
    delta[4] = rotation.z;
    delta[6] = -rotation.x;
    delta[8] = -rotation.y;
    delta[9] = rotation.x;
    return delta;
  }
  const double kx = rotation.x / angle;
  const double ky = rotation.y / angle;
  const double kz = rotation.z / angle;
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  const double one_minus_cosine = 1.0 - cosine;
  delta[0] = cosine + one_minus_cosine * kx * kx;
  delta[1] = -sine * kz + one_minus_cosine * kx * ky;
  delta[2] = sine * ky + one_minus_cosine * kx * kz;
  delta[4] = sine * kz + one_minus_cosine * ky * kx;
  delta[5] = cosine + one_minus_cosine * ky * ky;
  delta[6] = -sine * kx + one_minus_cosine * ky * kz;
  delta[8] = -sine * ky + one_minus_cosine * kz * kx;
  delta[9] = sine * kx + one_minus_cosine * kz * ky;
  delta[10] = cosine + one_minus_cosine * kz * kz;
  return delta;
}

// Projects the rotation block onto SO(3) so the final transform satisfies the public contract's
// orthonormality and determinant tolerances despite accumulated floating-point drift.
[[nodiscard]] Mat4 orthonormalize(const Mat4& transform) noexcept {
  Eigen::Matrix3d rotation;
  rotation << transform[0], transform[1], transform[2], transform[4], transform[5], transform[6],
      transform[8], transform[9], transform[10];
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d& left = svd.matrixU();
  const Eigen::Matrix3d& right = svd.matrixV();
  const double determinant = (left * right.transpose()).determinant();
  Eigen::Matrix3d sign = Eigen::Matrix3d::Identity();
  if (determinant < 0.0) {
    sign(2, 2) = -1.0;
  }
  const Eigen::Matrix3d corrected = left * sign * right.transpose();
  Mat4 result = transform;
  result[0] = corrected(0, 0);
  result[1] = corrected(0, 1);
  result[2] = corrected(0, 2);
  result[4] = corrected(1, 0);
  result[5] = corrected(1, 1);
  result[6] = corrected(1, 2);
  result[8] = corrected(2, 0);
  result[9] = corrected(2, 1);
  result[10] = corrected(2, 2);
  return result;
}

[[nodiscard]] Error backend_error(std::string reason, ErrorCode code) {
  return Error{code,
               PipelineStage::registration,
               "PCL point-to-plane registration failed",
               {{"reason", std::move(reason)}}};
}

template <typename Function> void for_each_logical_index(SurfaceView surface, Function&& function) {
  if (!surface.grid()) {
    for (std::size_t index = 0; index < surface.storage_size(); ++index) {
      function(index);
    }
    return;
  }
  const auto& grid = *surface.grid();
  for (std::size_t row = 0; row < grid.height; ++row) {
    for (std::size_t column = 0; column < grid.width; ++column) {
      function(row * static_cast<std::size_t>(grid.row_stride) + column);
    }
  }
}

[[nodiscard]] bool is_valid_index(SurfaceView surface, std::size_t index) noexcept {
  return surface.valid().empty() || surface.valid()[index] == 1U;
}

struct ValidatedCloud final {
  std::vector<V3> points;
  std::vector<V3> normals;
};

[[nodiscard]] ValidatedCloud extract_valid(SurfaceView surface) {
  ValidatedCloud cloud;
  cloud.points.reserve(surface.size());
  cloud.normals.reserve(surface.size());
  const bool has_normals = !surface.normals().empty();
  for_each_logical_index(surface, [&](std::size_t index) {
    if (!is_valid_index(surface, index)) {
      return;
    }
    const Vec3f point = surface.points()[index];
    cloud.points.push_back(
        {static_cast<double>(point.x), static_cast<double>(point.y), static_cast<double>(point.z)});
    if (has_normals) {
      const Vec3f normal = surface.normals()[index];
      cloud.normals.push_back(
          normalized({static_cast<double>(normal.x), static_cast<double>(normal.y),
                      static_cast<double>(normal.z)}));
    }
  });
  return cloud;
}

} // namespace

Result<RegistrationMetrics> align_point_to_plane(SurfaceView reference, SurfaceView scan,
                                                 const RigidTransform& initial_transform,
                                                 RegistrationParameters parameters) noexcept {
  try {
    if (reference.normals().empty()) {
      return Result<RegistrationMetrics>::failure(backend_error(
          "reference normals are required for point-to-plane ICP", ErrorCode::invalid_input));
    }
    const auto reference_cloud = extract_valid(reference);
    const auto scan_cloud = extract_valid(scan);
    if (reference_cloud.points.empty() || scan_cloud.points.empty()) {
      return Result<RegistrationMetrics>::failure(
          backend_error("registration requires at least one valid point on each surface",
                        ErrorCode::invalid_input));
    }
    const bool scan_has_normals = !scan.normals().empty();

    auto reference_pcl = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    reference_pcl->points.reserve(reference_cloud.points.size());
    for (const V3 point : reference_cloud.points) {
      reference_pcl->points.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y),
                                         static_cast<float>(point.z));
    }
    reference_pcl->width = static_cast<std::uint32_t>(reference_pcl->points.size());
    reference_pcl->height = 1U;
    reference_pcl->is_dense = true;

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(reference_pcl);

    const auto& initial = initial_transform.matrix();
    Mat4 current = initial;
    const double max_correspondence_distance = parameters.max_correspondence_distance_mm;
    const double huber_delta = parameters.huber_delta_mm;
    const double rotation_epsilon = parameters.rotation_epsilon_rad;
    const double translation_epsilon = parameters.translation_epsilon_mm;
    const double residual_epsilon = parameters.residual_epsilon_mm;

    std::uint32_t completed_iterations = 0U;
    bool converged = false;
    bool degenerate = false;
    double previous_mean_residual = std::numeric_limits<double>::infinity();

    pcl::PointXYZ query;
    std::vector<int> neighbor_indices(1);
    std::vector<float> squared_distances(1);
    Eigen::Matrix<double, 6, 6> normal_matrix;
    Eigen::Matrix<double, 6, 1> gradient;

    for (std::uint32_t iteration = 0; iteration < parameters.max_iterations; ++iteration) {
      normal_matrix.setZero();
      gradient.setZero();
      double residual_sum = 0.0;
      std::uint64_t correspondence_count = 0U;

      for (std::size_t scan_index = 0; scan_index < scan_cloud.points.size(); ++scan_index) {
        const V3 transformed = apply_point(current, scan_cloud.points[scan_index]);
        query.x = static_cast<float>(transformed.x);
        query.y = static_cast<float>(transformed.y);
        query.z = static_cast<float>(transformed.z);
        if (tree.nearestKSearch(query, 1, neighbor_indices, squared_distances) == 0) {
          continue;
        }
        const std::size_t reference_index = static_cast<std::size_t>(neighbor_indices[0]);
        const double distance = std::sqrt(static_cast<double>(squared_distances[0]));
        if (distance > max_correspondence_distance) {
          continue;
        }
        const V3 reference_normal = reference_cloud.normals[reference_index];
        if (scan_has_normals) {
          const V3 transformed_normal = apply_direction(current, scan_cloud.normals[scan_index]);
          if (dot(transformed_normal, reference_normal) < 0.0) {
            continue; // Reject back-facing correspondences.
          }
        }
        const V3 difference = transformed - reference_cloud.points[reference_index];
        const double residual = dot(reference_normal, difference);
        const double absolute_residual = std::abs(residual);
        const double weight =
            absolute_residual <= huber_delta ? 1.0 : huber_delta / absolute_residual;
        const V3 lever = cross(transformed, reference_normal);
        Eigen::Matrix<double, 1, 6> jacobian;
        jacobian << lever.x, lever.y, lever.z, reference_normal.x, reference_normal.y,
            reference_normal.z;
        normal_matrix += weight * jacobian.transpose() * jacobian;
        gradient += weight * (-residual) * jacobian.transpose();
        residual_sum += absolute_residual;
        ++correspondence_count;
      }

      const double mean_residual = correspondence_count == 0U
                                       ? 0.0
                                       : residual_sum / static_cast<double>(correspondence_count);
      if (iteration > 0U && correspondence_count > 0U) {
        const double improvement = previous_mean_residual - mean_residual;
        if (std::isfinite(improvement) && improvement >= 0.0 && improvement < residual_epsilon) {
          converged = true;
          break;
        }
      }
      previous_mean_residual = mean_residual;

      if (correspondence_count < 6U) {
        degenerate = true;
        break;
      }

      Eigen::Matrix<double, 6, 1> twist = normal_matrix.fullPivLu().solve(gradient);
      if (!twist.allFinite()) {
        degenerate = true;
        break;
      }

      const V3 rotation{twist(0), twist(1), twist(2)};
      const V3 translation{twist(3), twist(4), twist(5)};
      current = multiply(delta_from_twist(rotation, translation), current);
      ++completed_iterations;

      if (norm(rotation) < rotation_epsilon && norm(translation) < translation_epsilon) {
        converged = true;
        break;
      }
    }

    const Mat4 final_transform = orthonormalize(current);
    auto validated = RigidTransform::create(final_transform, initial_transform.source_frame(),
                                            initial_transform.target_frame(), 1.0e-6);
    if (!validated) {
      return Result<RegistrationMetrics>::failure(std::move(validated).error());
    }

    // Re-evaluate correspondences under the final transform so the reported metrics describe the
    // accepted inlier set rather than the last incremental update.
    double squared_residual_sum = 0.0;
    std::uint64_t valid_pairs = 0U;
    for (std::size_t scan_index = 0; scan_index < scan_cloud.points.size(); ++scan_index) {
      const V3 transformed = apply_point(final_transform, scan_cloud.points[scan_index]);
      query.x = static_cast<float>(transformed.x);
      query.y = static_cast<float>(transformed.y);
      query.z = static_cast<float>(transformed.z);
      if (tree.nearestKSearch(query, 1, neighbor_indices, squared_distances) == 0) {
        continue;
      }
      const std::size_t reference_index = static_cast<std::size_t>(neighbor_indices[0]);
      if (std::sqrt(static_cast<double>(squared_distances[0])) > max_correspondence_distance) {
        continue;
      }
      const V3 reference_normal = reference_cloud.normals[reference_index];
      if (scan_has_normals) {
        const V3 transformed_normal =
            apply_direction(final_transform, scan_cloud.normals[scan_index]);
        if (dot(transformed_normal, reference_normal) < 0.0) {
          continue;
        }
      }
      const double residual =
          dot(reference_normal, transformed - reference_cloud.points[reference_index]);
      squared_residual_sum += residual * residual;
      ++valid_pairs;
    }

    const double fitness =
        scan_cloud.points.empty()
            ? 0.0
            : static_cast<double>(valid_pairs) / static_cast<double>(scan_cloud.points.size());
    const double inlier_rmse =
        valid_pairs == 0U ? 0.0
                          : std::sqrt(squared_residual_sum / static_cast<double>(valid_pairs));

    const Mat4 relative = multiply(final_transform, inverse_rigid(initial));
    const V3 relative_translation{relative[3], relative[7], relative[11]};
    const double translation_delta = norm(relative_translation);
    const double rotation_trace = relative[0] + relative[5] + relative[10];
    const double clamped_cosine = std::clamp((rotation_trace - 1.0) / 2.0, -1.0, 1.0);
    const double rotation_delta = std::acos(clamped_cosine) * 180.0 / kPi;

    const RegistrationConvergence convergence =
        degenerate ? RegistrationConvergence::degenerate_input
                   : (converged ? RegistrationConvergence::converged
                                : RegistrationConvergence::not_converged);

    return RegistrationMetrics::create(std::move(validated).value(), convergence,
                                       completed_iterations, valid_pairs, fitness, inlier_rmse,
                                       translation_delta, rotation_delta);
  } catch (const std::exception& exception) {
    return Result<RegistrationMetrics>::failure(
        backend_error(exception.what(), ErrorCode::internal_error));
  } catch (...) {
    return Result<RegistrationMetrics>::failure(
        backend_error("unknown backend exception", ErrorCode::internal_error));
  }
}

} // namespace pointcloud_ad::backends::pcl_backend
