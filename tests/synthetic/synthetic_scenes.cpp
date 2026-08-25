#include "synthetic_scenes.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace pointcloud_ad::synthetic {
namespace {

constexpr double kPi = 3.14159265358979323846;

const FrameId kFixtureFrame = FrameId::create("fixture").value();
const FrameId kScannerFrame = FrameId::create("scanner").value();

[[nodiscard]] OwnedSurface make_plane_patch(const FrameId& frame, double half_span, double step,
                                            double amplitude, double sigma, double center_x,
                                            double center_y, double normal_z) {
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  const double two_sigma_squared = 2.0 * sigma * sigma;
  for (double x = -half_span; x <= half_span + 1.0e-9; x += step) {
    for (double y = -half_span; y <= half_span + 1.0e-9; y += step) {
      const double dx = x - center_x;
      const double dy = y - center_y;
      const double z = amplitude * std::exp(-(dx * dx + dy * dy) / two_sigma_squared);
      points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      normals.push_back({0.0F, 0.0F, static_cast<float>(normal_z)});
    }
  }
  return OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                              LengthUnit::millimeter, frame)
      .value();
}

[[nodiscard]] OwnedSurface make_feature_patch(const FrameId& frame) {
  // Two offset Gaussian bumps provide non-rotationally-symmetric features for the ICP solver while
  // analytic normals keep the point-to-plane residual exact.
  struct Feature {
    double center_x;
    double center_y;
    double amplitude;
    double sigma;
  };
  constexpr Feature features[] = {{1.5, 0.0, 0.4, 0.7}, {-1.0, -1.0, 0.3, 0.6}};
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  for (double x = -6.0; x <= 6.0 + 1.0e-9; x += 0.5) {
    for (double y = -6.0; y <= 6.0 + 1.0e-9; y += 0.5) {
      double z = 0.0;
      double dz_dx = 0.0;
      double dz_dy = 0.0;
      for (const auto& feature : features) {
        const double dx = x - feature.center_x;
        const double dy = y - feature.center_y;
        const double two_sigma_squared = 2.0 * feature.sigma * feature.sigma;
        const double decay = std::exp(-(dx * dx + dy * dy) / two_sigma_squared);
        z += feature.amplitude * decay;
        dz_dx += feature.amplitude * decay * (-dx / (feature.sigma * feature.sigma));
        dz_dy += feature.amplitude * decay * (-dy / (feature.sigma * feature.sigma));
      }
      points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      const double magnitude = std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy + 1.0);
      normals.push_back({static_cast<float>(-dz_dx / magnitude),
                         static_cast<float>(-dz_dy / magnitude),
                         static_cast<float>(1.0 / magnitude)});
    }
  }
  return OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                              LengthUnit::millimeter, frame)
      .value();
}

[[nodiscard]] Vec3d rotate_only(const RigidTransform& transform, Vec3d vector) {
  const auto& m = transform.matrix();
  return Vec3d{m[0] * vector.x + m[1] * vector.y + m[2] * vector.z,
               m[4] * vector.x + m[5] * vector.y + m[6] * vector.z,
               m[8] * vector.x + m[9] * vector.y + m[10] * vector.z};
}

// Applies a rigid transform to every point and rotates every normal, producing a new surface in the
// given frame. The caller passes the frame the transformed coordinates are expressed in.
[[nodiscard]] OwnedSurface apply_rigid(SurfaceView surface, const RigidTransform& transform,
                                       const FrameId& frame) {
  const auto source = surface.points();
  const auto source_normals = surface.normals();
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  points.reserve(source.size());
  normals.reserve(source_normals.size());
  for (std::size_t index = 0; index < source.size(); ++index) {
    const auto transformed = transform.apply(Vec3d{source[index].x, source[index].y,
                                                   source[index].z});
    points.push_back({static_cast<float>(transformed.x), static_cast<float>(transformed.y),
                      static_cast<float>(transformed.z)});
    if (!source_normals.empty()) {
      const auto rotated = rotate_only(transform, Vec3d{source_normals[index].x,
                                                        source_normals[index].y,
                                                        source_normals[index].z});
      normals.push_back({static_cast<float>(rotated.x), static_cast<float>(rotated.y),
                         static_cast<float>(rotated.z)});
    }
  }
  return OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                              surface.unit(), frame)
      .value();
}

// Removes every point inside the horizontal disk centred on (cx, cy) with the given radius.
[[nodiscard]] OwnedSurface remove_disk(SurfaceView surface, double cx, double cy, double radius) {
  const auto source = surface.points();
  const auto source_normals = surface.normals();
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  points.reserve(source.size());
  normals.reserve(source_normals.size());
  for (std::size_t index = 0; index < source.size(); ++index) {
    const double dx = static_cast<double>(source[index].x) - cx;
    const double dy = static_cast<double>(source[index].y) - cy;
    if (dx * dx + dy * dy <= radius * radius) {
      continue;
    }
    points.push_back(source[index]);
    if (!source_normals.empty()) {
      normals.push_back(source_normals[index]);
    }
  }
  return OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                              surface.unit(), surface.frame())
      .value();
}

// Keeps only the points whose x coordinate is below the threshold, simulating a partial scan.
[[nodiscard]] OwnedSurface keep_half_plane(SurfaceView surface, double x_threshold) {
  const auto source = surface.points();
  const auto source_normals = surface.normals();
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  points.reserve(source.size());
  normals.reserve(source_normals.size());
  for (std::size_t index = 0; index < source.size(); ++index) {
    if (static_cast<double>(source[index].x) > x_threshold) {
      continue;
    }
    points.push_back(source[index]);
    if (!source_normals.empty()) {
      normals.push_back(source_normals[index]);
    }
  }
  return OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                              surface.unit(), surface.frame())
      .value();
}

[[nodiscard]] SyntheticScene scene(std::string name, OwnedSurface reference, OwnedSurface scan,
                                   SceneTruth truth = {}) {
  return SyntheticScene{std::move(name), std::move(reference), std::move(scan), std::move(truth)};
}

} // namespace

SyntheticScene make_identical() {
  auto reference = make_plane_patch(kFixtureFrame, 6.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1.0);
  auto scan = make_plane_patch(kScannerFrame, 6.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1.0);
  return scene("synthetic_identical", std::move(reference), std::move(scan));
}

SyntheticScene make_rigid() {
  // A patch with two offset bumps gives the ICP solver non-rotationally-symmetric features and
  // analytic normals; a flat plane rotated in-plane is degenerate for ICP.
  auto reference = make_feature_patch(kFixtureFrame);
  const double angle = 2.0 * kPi / 180.0;
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  // Truth pose: scan->reference, 2 degrees about z plus a sub-millimetre translation, well inside
  // the registration gate's initial-pose bounds.
  const std::array<double, 16> matrix{cosine, -sine, 0.0, 0.5, sine, cosine, 0.0, 0.3,
                                      0.0,    0.0,   1.0, 0.1, 0.0,  0.0,   0.0, 1.0};
  const auto pose = RigidTransform::create(matrix, kScannerFrame, kFixtureFrame).value();
  // The scan is expressed in scanner-frame coordinates, so it is generated with the inverse pose
  // (fixture->scanner); ICP then recovers the scan->reference pose, matching `pose`.
  const double inverse_tx = -(cosine * 0.5 + sine * 0.3);
  const double inverse_ty = -(-sine * 0.5 + cosine * 0.3);
  const double inverse_tz = -0.1;
  const std::array<double, 16> inverse_matrix{cosine, sine, 0.0, inverse_tx, -sine, cosine,
                                              0.0,    inverse_ty, 0.0, 0.0, 1.0, inverse_tz,
                                              0.0,    0.0, 0.0, 1.0};
  const auto inverse_pose =
      RigidTransform::create(inverse_matrix, kFixtureFrame, kScannerFrame).value();
  auto scan = apply_rigid(reference.view(), inverse_pose, kScannerFrame);
  SceneTruth truth;
  truth.ground_truth_pose = pose;
  return scene("synthetic_rigid", std::move(reference), std::move(scan), std::move(truth));
}

SyntheticScene make_bump() {
  auto reference = make_plane_patch(kFixtureFrame, 6.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1.0);
  // A narrow bump keeps the ICP flattening small (roughly the bump's mean height), so the measured
  // depth stays within the 5% acceptance tolerance of the injected depth.
  auto scan = make_plane_patch(kScannerFrame, 6.0, 0.5, 0.8, 1.0, 0.0, 0.0, 1.0);
  SceneTruth truth;
  truth.defect_center_mm = Vec3d{0.0, 0.0, 0.8};
  truth.injected_depth_mm = 0.8;
  return scene("synthetic_bump", std::move(reference), std::move(scan), std::move(truth));
}

SyntheticScene make_dent() {
  auto reference = make_plane_patch(kFixtureFrame, 6.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1.0);
  auto scan = make_plane_patch(kScannerFrame, 6.0, 0.5, -0.8, 1.0, 0.0, 0.0, 1.0);
  SceneTruth truth;
  truth.defect_center_mm = Vec3d{0.0, 0.0, -0.8};
  truth.injected_depth_mm = -0.8;
  return scene("synthetic_dent", std::move(reference), std::move(scan), std::move(truth));
}

SyntheticScene make_missing() {
  // The removed block is wider than twice the comparison search radius so the inner core is
  // guaranteed to have no scan neighbour; the outer rim stays covered and is excluded from recall.
  auto reference = make_plane_patch(kFixtureFrame, 6.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1.0);
  auto scan = remove_disk(reference.view(), 0.0, 0.0, 3.0);
  SceneTruth truth;
  truth.missing_center_mm = Vec3d{0.0, 0.0, 0.0};
  truth.missing_radius_mm = 3.0;
  truth.missing_core_radius_mm = 1.5;
  return scene("synthetic_missing", std::move(reference), std::move(scan), std::move(truth));
}

SyntheticScene make_dropout() {
  auto reference = make_plane_patch(kFixtureFrame, 6.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1.0);
  auto scan = keep_half_plane(reference.view(), 0.0);
  return scene("synthetic_dropout", std::move(reference), std::move(scan));
}

SyntheticScene make_bad_pose() {
  auto reference = make_plane_patch(kFixtureFrame, 6.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1.0);
  auto scan = make_plane_patch(kScannerFrame, 6.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1.0);
  // The scan is shifted 100 mm along the normal so no correspondence exists within the ICP search
  // radius and the registration gate must reject the solve.
  const std::array<double, 16> matrix{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                      0.0, 0.0, 1.0, 100.0, 0.0, 0.0, 0.0, 1.0};
  const auto pose = RigidTransform::create(matrix, kScannerFrame, kFixtureFrame).value();
  auto displaced = apply_rigid(scan.view(), pose, kScannerFrame);
  SceneTruth truth;
  truth.ground_truth_pose = pose;
  return scene("registration_bad_pose", std::move(reference), std::move(displaced), std::move(truth));
}

SyntheticScene make_normal_orientation() {
  // The scan geometry matches the reference but its normals point against the reference normals,
  // so signed-deviation evidence is unreliable and the pipeline must not report PASS.
  auto reference = make_plane_patch(kFixtureFrame, 6.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1.0);
  auto scan = make_plane_patch(kScannerFrame, 6.0, 0.5, 0.0, 1.0, 0.0, 0.0, -1.0);
  return scene("normal_orientation", std::move(reference), std::move(scan));
}

SyntheticScene make_boundary_mask() {
  // A small bump sits near the patch edge so the boundary mask must suppress it as a false region.
  auto reference = make_plane_patch(kFixtureFrame, 6.0, 0.5, 0.0, 1.0, 0.0, 0.0, 1.0);
  auto scan = make_plane_patch(kScannerFrame, 6.0, 0.5, 0.5, 0.4, 5.5, 0.0, 1.0);
  SceneTruth truth;
  truth.defect_center_mm = Vec3d{5.5, 0.0, 0.5};
  truth.injected_depth_mm = 0.5;
  return scene("boundary_mask", std::move(reference), std::move(scan), std::move(truth));
}

} // namespace pointcloud_ad::synthetic
