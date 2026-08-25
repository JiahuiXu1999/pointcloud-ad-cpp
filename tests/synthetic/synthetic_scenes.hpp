#pragma once

#include <optional>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/surface.hpp>
#include <string>

namespace pointcloud_ad::synthetic {

// Ground truth recorded for one synthetic scene so the acceptance matrix can assert injected
// geometry, pose, and defect parameters against the pipeline report.
struct SceneTruth final {
  // scan->reference ground-truth pose for the rigid and bad-pose scenes.
  std::optional<RigidTransform> ground_truth_pose;
  // Dent/bump injection centre in the shared xy plane.
  std::optional<Vec3d> defect_center_mm;
  // Signed injected depth in millimetres (bump positive, dent negative).
  double injected_depth_mm{0.0};
  // Centre and radius of the scan block removed for the missing-material scene, plus the radius
  // of the guaranteed-uncovered core (block radius minus the comparison search radius).
  std::optional<Vec3d> missing_center_mm;
  double missing_radius_mm{0.0};
  double missing_core_radius_mm{0.0};
  // Half span and spacing of the generated planar patches.
  double half_span_mm{6.0};
  double step_mm{0.5};
};

struct SyntheticScene final {
  std::string name;
  OwnedSurface reference;
  OwnedSurface scan;
  SceneTruth truth;
};

// Scene factories. The names match the acceptance-matrix automation column. Inputs are always
// generated in millimetres with the reference in frame "fixture" and the scan in frame "scanner".
// Geometry is fully deterministic; no randomness is used so runs are reproducible by construction.
[[nodiscard]] SyntheticScene make_identical();
[[nodiscard]] SyntheticScene make_rigid();
[[nodiscard]] SyntheticScene make_bump();
[[nodiscard]] SyntheticScene make_dent();
[[nodiscard]] SyntheticScene make_missing();
[[nodiscard]] SyntheticScene make_dropout();
[[nodiscard]] SyntheticScene make_bad_pose();
[[nodiscard]] SyntheticScene make_normal_orientation();
[[nodiscard]] SyntheticScene make_boundary_mask();
[[nodiscard]] SyntheticScene make_input_validation();

} // namespace pointcloud_ad::synthetic
