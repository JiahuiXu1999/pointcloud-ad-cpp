#pragma once
#include <cmath>
#include <pointcloud_ad/inspection_pipeline.hpp>
#include <pointcloud_ad/registration_engine.hpp>
#include <vector>
namespace pointcloud_ad::performance {
[[nodiscard]] InspectionConfig make_config() {
  InspectionConfig config;
  config.schema_version = "1.0";
  config.profile = "synthetic_demo";
  config.input.reference_unit = LengthUnit::millimeter;
  config.input.scan_unit = LengthUnit::millimeter;
  config.input.reference_frame = FrameId::create("fixture").value();
  config.input.scan_frame = FrameId::create("scanner").value();
  config.preprocess.voxel_size_mm = 0.2;
  config.preprocess.normal_radius_mm = 1.0;
  config.preprocess.normal_min_neighbors = 12;
  config.preprocess.boundary_radius_mm = 0.8;
  config.registration.max_iterations = 60;
  config.registration.max_correspondence_distance_mm = 1.0;
  config.registration.huber_delta_mm = 0.3;
  config.registration.translation_epsilon_mm = 0.0001;
  config.registration.rotation_epsilon_rad = 0.00001;
  config.registration.residual_epsilon_mm = 0.00001;
  config.registration_gate.min_overlap_ratio = 0.7;
  config.registration_gate.max_inlier_rmse_mm = 0.2;
  config.registration_gate.min_valid_pairs = 500;
  config.registration_gate.max_translation_from_initial_mm = 5.0;
  config.registration_gate.max_rotation_from_initial_deg = 5.0;
  config.comparison.max_search_distance_mm = 0.8;
  config.comparison.max_normal_angle_deg = 35.0;
  config.comparison.boundary_exclusion_mm = 0.6;
  config.comparison.min_valid_coverage_ratio = 0.75;
  config.detection.positive_threshold_mm = 0.25;
  config.detection.negative_threshold_mm = -0.25;
  config.detection.cluster_tolerance_mm = 0.6;
  config.detection.min_cluster_points = 20;
  config.detection.measurement_error_budget_mm = 0.0;
  config.execution.deterministic = true;
  config.execution.thread_count = 1;
  config.execution.random_seed = 5489;
  return config;
}

inline OwnedSurface patch(const FrameId& frame, int side, bool shifted, bool curved) {
  std::vector<Vec3f> points, normals;
  for (int row = 0; row < side; ++row) {
    for (int col = 0; col < side; ++col) {
      const double x = (row - side / 2) * 0.25, y = (col - side / 2) * 0.25;
      const double z = curved ? 0.008 * x * x + 0.013 * y * y : 0.0;
      const double nx = curved ? -0.016 * x : 0.0, ny = curved ? -0.026 * y : 0.0;
      const double len = std::sqrt(nx * nx + ny * ny + 1.0);
      points.push_back({static_cast<float>(x - (shifted ? 0.003 : 0.0)),
                        static_cast<float>(y + (shifted ? 0.002 : 0.0)),
                        static_cast<float>(z - (shifted ? 0.01 : 0.0))});
      normals.push_back({static_cast<float>(nx / len), static_cast<float>(ny / len),
                         static_cast<float>(1.0 / len)});
    }
  }
  return OwnedSurface::create(std::move(points), std::move(normals), {}, {}, LengthUnit::millimeter,
                              frame)
      .value();
}
inline RegistrationParameters parameters(RegistrationMethod method) {
  RegistrationParameters p{40, 0.2, 0.1, 1e-7, 1e-8, 1e-9};
  p.method = method;
  return p;
}
inline RigidTransform initial(const FrameId& scan, const FrameId& reference) {
  return RigidTransform::create({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, scan, reference)
      .value();
}
} // namespace pointcloud_ad::performance
