#include "pcl_registration_backend.hpp"
#include "registration_gate.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/registration.hpp>
#include <pointcloud_ad/surface.hpp>
#include <string_view>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

using pointcloud_ad::FrameId;
using pointcloud_ad::LengthUnit;
using pointcloud_ad::OwnedSurface;
using pointcloud_ad::RegistrationConvergence;
using pointcloud_ad::RegistrationParameters;
using pointcloud_ad::RigidTransform;
using pointcloud_ad::SurfaceView;
using pointcloud_ad::Vec3d;
using pointcloud_ad::Vec3f;

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

[[nodiscard]] std::array<double, 16> rotation_z(double degrees) noexcept {
  const double radians = degrees * kPi / 180.0;
  const double cosine = std::cos(radians);
  const double sine = std::sin(radians);
  return {cosine, -sine, 0.0, 0.0, sine, cosine, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
}

[[nodiscard]] std::array<double, 16> make_transform(const std::array<double, 16>& rotation,
                                                    double tx, double ty, double tz) noexcept {
  std::array<double, 16> result = rotation;
  result[3] = tx;
  result[7] = ty;
  result[11] = tz;
  return result;
}

[[nodiscard]] std::array<double, 16>
inverse_rigid(const std::array<double, 16>& transform) noexcept {
  const double r00 = transform[0], r01 = transform[1], r02 = transform[2];
  const double r10 = transform[4], r11 = transform[5], r12 = transform[6];
  const double r20 = transform[8], r21 = transform[9], r22 = transform[10];
  const double tx = transform[3], ty = transform[7], tz = transform[11];
  std::array<double, 16> inverse{};
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

[[nodiscard]] Vec3d apply_point(const std::array<double, 16>& transform, Vec3d point) noexcept {
  return {transform[0] * point.x + transform[1] * point.y + transform[2] * point.z + transform[3],
          transform[4] * point.x + transform[5] * point.y + transform[6] * point.z + transform[7],
          transform[8] * point.x + transform[9] * point.y + transform[10] * point.z +
              transform[11]};
}

[[nodiscard]] Vec3f rotate_normal(const std::array<double, 16>& transform, Vec3f normal) noexcept {
  return {static_cast<float>(transform[0] * normal.x + transform[1] * normal.y +
                             transform[2] * normal.z),
          static_cast<float>(transform[4] * normal.x + transform[5] * normal.y +
                             transform[6] * normal.z),
          static_cast<float>(transform[8] * normal.x + transform[9] * normal.y +
                             transform[10] * normal.z)};
}

[[nodiscard]] double rotation_angle_degrees(const std::array<double, 16>& left,
                                            const std::array<double, 16>& right) noexcept {
  // Angle between the two rotation blocks equals acos((trace(R_left^T R_right) - 1) / 2), and
  // trace(R_left^T R_right) is the Frobenius inner product of the two 3x3 blocks.
  double inner_product = 0.0;
  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t column = 0; column < 3U; ++column) {
      inner_product += left[row * 4U + column] * right[row * 4U + column];
    }
  }
  const double cosine = std::clamp((inner_product - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cosine) * 180.0 / kPi;
}

struct SurfacePatch final {
  OwnedSurface surface;
  std::vector<Vec3d> points;
};

// Deterministic elliptic-paraboloid patch (z = a x^2 + b y^2 with a != b) with analytic outward
// unit normals. The unequal curvatures break rotational symmetry so all six degrees of freedom are
// observable to point-to-plane ICP, unlike a sphere or flat plane whose symmetric geometry leaves
// rotational/tangential motion unobservable.
[[nodiscard]] SurfacePatch make_patch(const FrameId& frame, double half_span, double step) {
  constexpr double curvature_x = 0.02;
  constexpr double curvature_y = 0.03;
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  std::vector<Vec3d> points_double;
  for (double x = -half_span; x <= half_span + 1.0e-9; x += step) {
    for (double y = -half_span; y <= half_span + 1.0e-9; y += step) {
      const double z = curvature_x * x * x + curvature_y * y * y;
      const double nx = -2.0 * curvature_x * x;
      const double ny = -2.0 * curvature_y * y;
      const double length = std::sqrt(nx * nx + ny * ny + 1.0);
      points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      normals.push_back({static_cast<float>(nx / length), static_cast<float>(ny / length),
                         static_cast<float>(1.0 / length)});
      points_double.push_back({x, y, z});
    }
  }
  auto surface = OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                                      LengthUnit::millimeter, frame);
  return SurfacePatch{std::move(surface).value(), std::move(points_double)};
}

RegistrationParameters icp_parameters() {
  RegistrationParameters parameters;
  parameters.max_iterations = 100U;
  parameters.max_correspondence_distance_mm = 2.0;
  parameters.huber_delta_mm = 0.5;
  parameters.translation_epsilon_mm = 1.0e-6;
  parameters.rotation_epsilon_rad = 1.0e-7;
  parameters.residual_epsilon_mm = 1.0e-7;
  return parameters;
}

} // namespace

int main() {
  using pointcloud_ad::backends::pcl_backend::align_point_to_plane;
  using pointcloud_ad::registration::evaluate_gate;

  constexpr std::array<double, 16> kIdentity{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                             0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  bool passed = true;

  auto reference_frame = FrameId::create("fixture").value();
  auto scan_frame = FrameId::create("scanner").value();

  const auto reference = make_patch(reference_frame, 8.0, 0.8);

  // Ground-truth scan-to-reference transform: 2 degrees about Z plus a small translation.
  const auto ground_truth = make_transform(rotation_z(2.0), 0.8, -0.6, 0.5);
  const auto ground_truth_inverse = inverse_rigid(ground_truth);
  const auto ground_truth_transform =
      RigidTransform::create(ground_truth, scan_frame, reference_frame).value();

  std::vector<Vec3f> scan_points;
  std::vector<Vec3f> scan_normals;
  scan_points.reserve(reference.surface.size());
  scan_normals.reserve(reference.surface.size());
  for (std::size_t index = 0; index < reference.points.size(); ++index) {
    const Vec3d mapped = apply_point(ground_truth_inverse, reference.points[index]);
    scan_points.push_back(
        {static_cast<float>(mapped.x), static_cast<float>(mapped.y), static_cast<float>(mapped.z)});
    scan_normals.push_back(
        rotate_normal(ground_truth_inverse, reference.surface.view().normals()[index]));
  }
  auto scan_surface = OwnedSurface::create(std::move(scan_points), std::move(scan_normals), {},
                                           std::nullopt, LengthUnit::millimeter, scan_frame)
                          .value();

  // AC-002: starting from identity, ICP must recover the ground-truth transform to within
  // 0.02 mm of translation and 0.02 degrees of rotation.
  const auto identity_transform =
      RigidTransform::create(kIdentity, scan_frame, reference_frame).value();
  auto solved = align_point_to_plane(reference.surface.view(), scan_surface.view(),
                                     identity_transform, icp_parameters());
  passed &=
      expect(static_cast<bool>(solved), "ICP must return metrics for a rigid-alignment input");
  if (solved) {
    const auto& metrics = solved.value();
    passed &= expect(metrics.convergence() == RegistrationConvergence::converged,
                     "ICP must converge for a rigid-alignment input");
    const auto& final = metrics.final_transform().matrix();
    const double translation_error =
        std::sqrt((final[3] - ground_truth[3]) * (final[3] - ground_truth[3]) +
                  (final[7] - ground_truth[7]) * (final[7] - ground_truth[7]) +
                  (final[11] - ground_truth[11]) * (final[11] - ground_truth[11]));
    const double rotation_error = rotation_angle_degrees(final, ground_truth);
    passed &= expect(translation_error <= 0.02, "ICP translation error must not exceed 0.02 mm");
    passed &= expect(rotation_error <= 0.02, "ICP rotation error must not exceed 0.02 degrees");
    if (translation_error > 0.02 || rotation_error > 0.02) {
      std::cerr << "  translation_error=" << translation_error
                << " rotation_error=" << rotation_error << '\n';
    }
  }

  // AC-007: a grossly wrong initial pose must be rejected by the registration gate rather than
  // proceeding to defect detection. A 30 mm translation leaves no correspondence within the 2 mm
  // search radius, so the solver reports a degenerate input.
  const auto bad_pose = make_transform(rotation_z(0.0), 30.0, 30.0, 30.0);
  const auto bad_pose_transform =
      RigidTransform::create(bad_pose, scan_frame, reference_frame).value();
  auto rejected_solve = align_point_to_plane(reference.surface.view(), scan_surface.view(),
                                             bad_pose_transform, icp_parameters());
  passed &= expect(static_cast<bool>(rejected_solve),
                   "ICP must still produce metrics for a bad-pose input");
  bool gate_rejected = false;
  if (rejected_solve) {
    pointcloud_ad::ValidatedRegistrationGateConfig gate{0.7, 0.2, 500, 5.0, 5.0};
    auto result = evaluate_gate(rejected_solve.value(), gate);
    passed &= expect(static_cast<bool>(result), "gate must evaluate a bad-pose solve");
    if (result) {
      gate_rejected = !result.value().passed;
    }
  }
  passed &= expect(gate_rejected, "the registration gate must reject a grossly wrong initial pose");

  return passed ? 0 : 1;
}
