#include "pcl_registration_backend.hpp"

#include "../../registration/cpu_dispatch.hpp"
#include "../../registration/cpu_executor.hpp"
#include "../../registration/cpu_kernels.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <memory>
#include <stdexcept>
#ifdef POINTCLOUDAD_WITH_CUDA
#include "cuda_correspondence.hpp"
#endif
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

using registration::CpuExecutor;

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Minimal double-precision rigid-motion primitives. Transforms are 4x4 row-major
// matrices, consistent with the public RigidTransform storage layout.
// ---------------------------------------------------------------------------

using V3 = Vec3d;

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
// orthonormality and determinant tolerances despite accumulated floating-point drift. ICP composes
// rigid increments, so modified Gram-Schmidt is sufficient here and avoids a full matrix SVD.
[[nodiscard]] Mat4 orthonormalize(const Mat4& transform) noexcept {
  const V3 first = normalized({transform[0], transform[4], transform[8]});
  const V3 second_source{transform[1], transform[5], transform[9]};
  const V3 second = normalized(second_source - dot(second_source, first) * first);
  const V3 third = normalized(cross(first, second));
  Mat4 result = transform;
  result[0] = first.x;
  result[4] = first.y;
  result[8] = first.z;
  result[1] = second.x;
  result[5] = second.y;
  result[9] = second.z;
  result[2] = third.x;
  result[6] = third.y;
  result[10] = third.z;
  return result;
}

[[nodiscard]] Error backend_error(std::string reason, ErrorCode code) {
  return Error{code,
               PipelineStage::registration,
               "registration backend failed",
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

namespace {

using Covariance = Eigen::Matrix3d;

[[nodiscard]] std::vector<Covariance> estimate_covariances(const ValidatedCloud& cloud,
                                                           RegistrationParameters parameters,
                                                           CpuExecutor& executor) {
  if (cloud.points.size() < parameters.covariance_neighbors) {
    throw std::invalid_argument(
        "GICP requires at least covariance_neighbors valid points per cloud");
  }
  auto points = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto point : cloud.points) {
    points->emplace_back(static_cast<float>(point.x), static_cast<float>(point.y),
                         static_cast<float>(point.z));
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(points);
  std::vector<Covariance> result(cloud.points.size());
  struct Scratch {
    std::vector<int> indices;
    std::vector<float> distances;
  };
  std::vector<Scratch> scratch(executor.concurrency());
  for (auto& item : scratch) {
    item.indices.resize(parameters.covariance_neighbors);
    item.distances.resize(parameters.covariance_neighbors);
  }
  executor.run(cloud.points.size(), [&](std::size_t block, std::size_t worker) {
    auto& [indices, distances] = scratch[worker];
    for (std::size_t point_index = block * CpuExecutor::block_size;
         point_index < std::min(cloud.points.size(), (block + 1) * CpuExecutor::block_size);
         ++point_index) {
      const auto& point = (*points)[point_index];
      if (tree.nearestKSearch(point, static_cast<int>(parameters.covariance_neighbors), indices,
                              distances) < static_cast<int>(parameters.covariance_neighbors)) {
        throw std::invalid_argument("GICP covariance neighborhood has insufficient support");
      }
      std::sort(indices.begin(), indices.end());
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (int index : indices) {
        const auto p = cloud.points[static_cast<std::size_t>(index)];
        mean += Eigen::Vector3d(p.x, p.y, p.z);
      }
      mean /= static_cast<double>(indices.size());
      Covariance covariance = Covariance::Zero();
      for (int index : indices) {
        const auto p = cloud.points[static_cast<std::size_t>(index)];
        const Eigen::Vector3d delta = Eigen::Vector3d(p.x, p.y, p.z) - mean;
        covariance += delta * delta.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Covariance> eigen(covariance);
      if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite() ||
          eigen.eigenvalues()(1) <= 1.0e-12) {
        throw std::invalid_argument("GICP covariance neighborhood is collinear or coincident");
      }
      result[point_index] = eigen.eigenvectors() *
                            Eigen::Vector3d(parameters.covariance_epsilon, 1.0, 1.0).asDiagonal() *
                            eigen.eigenvectors().transpose();
    }
  });
  return result;
}

class Search final {
public:
  Search(const ValidatedCloud& reference, std::size_t query_count, ComputeBackend backend,
         CpuExecutor& executor)
      : executor_(executor), scratch_(executor.concurrency()), backend_(backend),
        points_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()) {
    packed_.reserve(query_count);
    result_.reserve(query_count);
    for (const auto p : reference.points) {
      points_->emplace_back(static_cast<float>(p.x), static_cast<float>(p.y),
                            static_cast<float>(p.z));
    }
    if (backend == ComputeBackend::cpu) {
      tree_.setInputCloud(points_);
    } else {
#ifdef POINTCLOUDAD_WITH_CUDA
      std::vector<Vec3f> packed;
      packed.reserve(reference.points.size());
      for (const auto& p : *points_) {
        packed.push_back({p.x, p.y, p.z});
      }
      gpu_ = std::make_unique<cuda_backend::CorrespondenceSearch>(packed, query_count);
#else
      (void)query_count;
      throw std::invalid_argument("GPU support is not compiled into this binary");
#endif
    }
  }

  [[nodiscard]] const std::vector<std::int32_t>& query(const std::vector<V3>& transformed,
                                                       double bound) {
    auto& packed = packed_;
    packed.clear();
    for (const auto p : transformed) {
      const Vec3f q{static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
      if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
        throw std::invalid_argument("transformed coordinate exceeds search precision range");
      }
      packed.push_back(q);
    }
    if (backend_ == ComputeBackend::gpu) {
#ifdef POINTCLOUDAD_WITH_CUDA
      gpu_->query_into(packed, bound, result_);
      return result_;
#else
      throw std::invalid_argument("GPU support is not compiled into this binary");
#endif
    }
    auto& result = result_;
    result.assign(packed.size(), -1);
    executor_.run(packed.size(), [&](std::size_t block, std::size_t worker) {
      auto& [indices, distances, ties, tie_distances] = scratch_[worker];
      for (std::size_t i = block * CpuExecutor::block_size;
           i < std::min(packed.size(), (block + 1) * CpuExecutor::block_size); ++i) {
        const pcl::PointXYZ q(packed[i].x, packed[i].y, packed[i].z);
        if (tree_.nearestKSearch(q, 1, indices, distances) == 0) {
          continue;
        }
        // Recheck near-equal float tree candidates in double, with the same stable tie rule as
        // CUDA.
        const double radius =
            std::sqrt(static_cast<double>(distances[0])) * (1.0 + 1.0e-6) + 1.0e-6;
        tree_.radiusSearch(q, radius, ties, tie_distances);
        double best = bound * bound;
        for (int candidate : ties) {
          const auto& p = (*points_)[static_cast<std::size_t>(candidate)];
          const double dx = static_cast<double>(q.x) - p.x;
          const double dy = static_cast<double>(q.y) - p.y;
          const double dz = static_cast<double>(q.z) - p.z;
          const double d = dx * dx + dy * dy + dz * dz;
          if (d < best || (d == best && (result[i] < 0 || candidate < result[i]))) {
            best = d;
            result[i] = candidate;
          }
        }
      }
    });
    return result;
  }

private:
  struct QueryScratch {
    std::vector<int> indices = std::vector<int>(1);
    std::vector<float> distances = std::vector<float>(1);
    std::vector<int> ties;
    std::vector<float> tie_distances;
  };
  CpuExecutor& executor_;
  std::vector<QueryScratch> scratch_;
  std::vector<Vec3f> packed_;
  std::vector<std::int32_t> result_;
  ComputeBackend backend_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
#ifdef POINTCLOUDAD_WITH_CUDA
  std::unique_ptr<cuda_backend::CorrespondenceSearch> gpu_;
#endif
};

void transform_points(const ValidatedCloud& cloud, const Mat4& transform, std::vector<V3>& points,
                      CpuExecutor& executor, CpuKernel kernel) {
  points.resize(cloud.points.size());
  const auto transform_function = registration::transform_kernel(kernel);
  executor.run(points.size(), [&](std::size_t block, std::size_t) {
    const auto begin = block * CpuExecutor::block_size;
    const auto count = std::min(CpuExecutor::block_size, points.size() - begin);
    transform_function(std::span<const Vec3d>(cloud.points).subspan(begin, count),
                       std::span<Vec3d>(points).subspan(begin, count), transform);
  });
}

} // namespace

namespace {
struct alignas(64) Accumulation {
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> matrix =
      Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero();
  Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
  double residual{};
  std::uint64_t count{};
  std::array<std::byte, 32> cache_padding{}; // Explicit 64-byte cache-line stride.
};
class Prepared final : public PreparedRegistration {
public:
  Prepared(SurfaceView reference, RegistrationParameters parameters, std::size_t max_scan_points,
           ComputeBackend backend)
      : kernel_(registration::select_cpu_kernel(parameters.cpu_kernel)),
        executor_(parameters.thread_count, std::max(max_scan_points, reference.size())),
        parameters_(parameters), reference_cloud_(extract_valid(reference)),
        search_(checked_reference(parameters), max_scan_points, backend, executor_) {
    transformed_.reserve(max_scan_points);
    reduction_.reserve(CpuExecutor::blocks(max_scan_points));
    if (parameters.method == RegistrationMethod::gicp) {
      reference_covariances_ = estimate_covariances(reference_cloud_, parameters, executor_);
    }
  }
  Result<RegistrationMetrics> align(SurfaceView scan,
                                    const RigidTransform& initial_transform) noexcept override {
    try {
      const auto parameters = parameters_;
      // Measured point-to-plane arithmetic favors the portable kernel on the baseline CPU.
      // Explicit AVX2 remains available; automatic transform dispatch still uses AVX2.
      const auto objective = registration::objective_kernel(
          parameters.cpu_kernel == CpuKernel::automatic &&
                  parameters.method == RegistrationMethod::point_to_plane
              ? CpuKernel::scalar
              : kernel_);
      const bool plane = parameters.method == RegistrationMethod::point_to_plane;
      const bool gicp = parameters.method == RegistrationMethod::gicp;
      const auto& reference_cloud = reference_cloud_;
      const auto& reference_covariances = reference_covariances_;
      auto& search = search_;
      const auto scan_cloud = extract_valid(scan);
      if (scan_cloud.points.empty()) {
        throw std::invalid_argument("registration requires valid scan points");
      }
      const auto scan_covariances = gicp ? estimate_covariances(scan_cloud, parameters, executor_)
                                         : std::vector<Covariance>{};
      const bool reject_back_facing =
          !reference_cloud.normals.empty() && !scan_cloud.normals.empty();
      const auto accepted = [&](std::size_t source, std::size_t target, const Mat4& pose) {
        return !reject_back_facing || dot(apply_direction(pose, scan_cloud.normals[source]),
                                          reference_cloud.normals[target]) >= 0.0;
      };
      const auto& initial = initial_transform.matrix();
      Mat4 current = initial;
      std::uint32_t completed_iterations = 0U;
      bool converged = false;
      bool degenerate = false;
      double previous_mean_residual = std::numeric_limits<double>::infinity();
      for (std::uint32_t iteration = 0; iteration < parameters.max_iterations; ++iteration) {
        transform_points(scan_cloud, current, transformed_, executor_, kernel_);
        const auto& transformed = transformed_;
        const auto& neighbors =
            search.query(transformed, parameters.max_correspondence_distance_mm);
        Covariance rotation;
        rotation << current[0], current[1], current[2], current[4], current[5], current[6],
            current[8], current[9], current[10];
        reduction_.assign(CpuExecutor::blocks(transformed.size()), Accumulation{});
        executor_.run(transformed.size(), [&](std::size_t block, std::size_t) {
          auto& normal_matrix = reduction_[block].matrix;
          auto& gradient = reduction_[block].gradient;
          auto& residual_sum = reduction_[block].residual;
          auto& correspondence_count = reduction_[block].count;
          for (std::size_t i = block * CpuExecutor::block_size;
               i < std::min(transformed.size(), (block + 1) * CpuExecutor::block_size); ++i) {
            if (neighbors[i] < 0) {
              continue;
            }
            const auto j = static_cast<std::size_t>(neighbors[i]);
            if (!accepted(i, j, current)) {
              continue;
            }
            registration::ObjectiveInput input;
            input.point = transformed[i];
            input.reference = reference_cloud.points[j];
            input.normal = plane ? reference_cloud.normals[j] : V3{};
            input.huber_delta = parameters.huber_delta_mm;
            input.plane = plane;
            input.generalized = gicp;
            if (gicp) {
              const Covariance information =
                  (reference_covariances[j] + rotation * scan_covariances[i] * rotation.transpose())
                      .inverse();
              for (std::size_t row = 0; row < 3; ++row) {
                for (std::size_t col = 0; col < 3; ++col) {
                  input.information[row * 3 + col] =
                      information(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col));
                }
              }
            }
            registration::ObjectiveContribution contribution;
            objective(input, contribution);
            normal_matrix += Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
                contribution.matrix.data());
            gradient += Eigen::Map<const Eigen::Matrix<double, 6, 1>>(contribution.gradient.data());
            residual_sum += contribution.absolute_residual;
            ++correspondence_count;
          }
        });
        Eigen::Matrix<double, 6, 6> normal_matrix = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
        double residual_sum = 0.0;
        std::uint64_t correspondence_count = 0;
        for (const auto& block : reduction_) {
          normal_matrix += block.matrix;
          gradient += block.gradient;
          residual_sum += block.residual;
          correspondence_count += block.count;
        }
        if (correspondence_count < 6U) {
          degenerate = true;
          break;
        }
        const auto decomposition = normal_matrix.fullPivLu();
        if (!plane && decomposition.rank() < 6) {
          degenerate = true;
          break;
        }
        const double mean_residual = residual_sum / static_cast<double>(correspondence_count);
        const double improvement = previous_mean_residual - mean_residual;
        if (iteration > 0U && std::isfinite(improvement) && improvement >= 0.0 &&
            improvement < parameters.residual_epsilon_mm) {
          converged = true;
          break;
        }
        previous_mean_residual = mean_residual;
        const Eigen::Matrix<double, 6, 1> twist = decomposition.solve(gradient);
        if (!twist.allFinite()) {
          degenerate = true;
          break;
        }
        const V3 rotation_delta{twist(0), twist(1), twist(2)};
        const V3 translation_delta{twist(3), twist(4), twist(5)};
        current = multiply(delta_from_twist(rotation_delta, translation_delta), current);
        ++completed_iterations;
        if (norm(rotation_delta) < parameters.rotation_epsilon_rad &&
            norm(translation_delta) < parameters.translation_epsilon_mm) {
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
      transform_points(scan_cloud, final_transform, transformed_, executor_, kernel_);
      const auto& transformed = transformed_;
      const auto& neighbors = search.query(transformed, parameters.max_correspondence_distance_mm);
      reduction_.assign(CpuExecutor::blocks(transformed.size()), Accumulation{});
      executor_.run(transformed.size(), [&](std::size_t block, std::size_t) {
        auto& squared_residual_sum = reduction_[block].residual;
        auto& valid_pairs = reduction_[block].count;
        for (std::size_t i = block * CpuExecutor::block_size;
             i < std::min(transformed.size(), (block + 1) * CpuExecutor::block_size); ++i) {
          if (neighbors[i] < 0) {
            continue;
          }
          const auto j = static_cast<std::size_t>(neighbors[i]);
          if (!accepted(i, j, final_transform)) {
            continue;
          }
          const auto d = transformed[i] - reference_cloud.points[j];
          // Preserve the original plane residual metric. Point-to-point and GICP report Euclidean
          // inlier RMSE in mm, never the dimensionless/regularized optimization objective.
          const double residual = plane ? dot(reference_cloud.normals[j], d) : norm(d);
          squared_residual_sum += residual * residual;
          ++valid_pairs;
        }
      });
      double squared_residual_sum = 0.0;
      std::uint64_t valid_pairs = 0;
      for (const auto& block : reduction_) {
        squared_residual_sum += block.residual;
        valid_pairs += block.count;
      }
      const double fitness =
          static_cast<double>(valid_pairs) / static_cast<double>(scan_cloud.points.size());
      const double rmse = valid_pairs == 0U
                              ? 0.0
                              : std::sqrt(squared_residual_sum / static_cast<double>(valid_pairs));
      const Mat4 relative = multiply(final_transform, inverse_rigid(initial));
      const double translation_delta = norm({relative[3], relative[7], relative[11]});
      const double cosine =
          std::clamp((relative[0] + relative[5] + relative[10] - 1.0) / 2.0, -1.0, 1.0);
      const auto convergence = degenerate ? RegistrationConvergence::degenerate_input
                                          : (converged ? RegistrationConvergence::converged
                                                       : RegistrationConvergence::not_converged);
      return RegistrationMetrics::create(std::move(validated).value(), convergence,
                                         completed_iterations, valid_pairs, fitness, rmse,
                                         translation_delta, std::acos(cosine) * 180.0 / kPi);
    } catch (const std::invalid_argument& exception) {
      return Result<RegistrationMetrics>::failure(
          backend_error(exception.what(), ErrorCode::invalid_input));
    } catch (const std::exception& exception) {
      return Result<RegistrationMetrics>::failure(
          backend_error(exception.what(), ErrorCode::internal_error));
    } catch (...) {
      return Result<RegistrationMetrics>::failure(
          backend_error("unknown backend exception", ErrorCode::internal_error));
    }
  }

private:
  const ValidatedCloud& checked_reference(RegistrationParameters parameters) const {
    if (reference_cloud_.points.empty()) {
      throw std::invalid_argument("registration requires valid reference points");
    }
    if (parameters.method == RegistrationMethod::point_to_plane) {
      if (reference_cloud_.normals.empty()) {
        throw std::invalid_argument("reference normals are required for point-to-plane ICP");
      }
      for (const auto normal : reference_cloud_.normals) {
        if (norm(normal) < 0.5) {
          throw std::invalid_argument("point-to-plane requires nonzero reference normals");
        }
      }
    }
    return reference_cloud_;
  }
  CpuKernel kernel_;
  std::vector<Accumulation> reduction_;
  CpuExecutor executor_;
  std::vector<V3> transformed_;
  RegistrationParameters parameters_;
  ValidatedCloud reference_cloud_;
  Search search_;
  std::vector<Covariance> reference_covariances_;
};
} // namespace

std::unique_ptr<PreparedRegistration> prepare_registration(SurfaceView reference,
                                                           RegistrationParameters parameters,
                                                           std::size_t max_scan_points,
                                                           ComputeBackend backend) {
  if (backend != ComputeBackend::cpu && backend != ComputeBackend::gpu) {
    throw std::invalid_argument("unsupported registration backend");
  }
  return std::make_unique<Prepared>(reference, parameters, max_scan_points, backend);
}

Result<RegistrationMetrics> align_registration(SurfaceView reference, SurfaceView scan,
                                               const RigidTransform& initial_transform,
                                               RegistrationParameters parameters,
                                               ComputeBackend backend) noexcept {
  try {
    auto input = RegistrationInput::create(reference, scan, initial_transform, parameters);
    if (!input) {
      return Result<RegistrationMetrics>::failure(std::move(input).error());
    }
    auto prepared = prepare_registration(reference, parameters, scan.size(), backend);
    return prepared->align(scan, initial_transform);
  } catch (const std::invalid_argument& exception) {
    return Result<RegistrationMetrics>::failure(
        backend_error(exception.what(), ErrorCode::invalid_input));
  } catch (const std::exception& exception) {
    return Result<RegistrationMetrics>::failure(
        backend_error(exception.what(), ErrorCode::internal_error));
  } catch (...) {
    return Result<RegistrationMetrics>::failure(
        backend_error("unknown backend exception", ErrorCode::internal_error));
  }
}

Result<RegistrationMetrics> align_point_to_plane(SurfaceView reference, SurfaceView scan,
                                                 const RigidTransform& initial_transform,
                                                 RegistrationParameters parameters) noexcept {
  parameters.method = RegistrationMethod::point_to_plane;
  return align_registration(reference, scan, initial_transform, parameters, ComputeBackend::cpu);
}

} // namespace pointcloud_ad::backends::pcl_backend
