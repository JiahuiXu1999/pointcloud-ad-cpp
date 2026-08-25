#include <cmath>
#include <cstdint>
#include <iostream>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/inspection_pipeline.hpp>
#include <pointcloud_ad/inspection_result.hpp>
#include <pointcloud_ad/surface.hpp>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using pointcloud_ad::FrameId;
using pointcloud_ad::InspectionConfig;
using pointcloud_ad::InspectionPipeline;
using pointcloud_ad::InspectionResult;
using pointcloud_ad::LengthUnit;
using pointcloud_ad::OwnedSurface;
using pointcloud_ad::Vec3f;
using pointcloud_ad::Verdict;

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

struct Patch final {
  OwnedSurface surface;
};

[[nodiscard]] Patch make_patch(const FrameId& frame, double half_span, double step,
                               double amplitude, double sigma) {
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  const double two_sigma_squared = 2.0 * sigma * sigma;
  for (double x = -half_span; x <= half_span + 1.0e-9; x += step) {
    for (double y = -half_span; y <= half_span + 1.0e-9; y += step) {
      const double z = amplitude * std::exp(-(x * x + y * y) / two_sigma_squared);
      points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      normals.push_back({0.0F, 0.0F, 1.0F});
    }
  }
  return Patch{OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                                    LengthUnit::millimeter, frame)
                   .value()};
}

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

} // namespace

int main() {
  bool passed = true;

  const auto reference_frame = FrameId::create("fixture").value();
  const auto scan_frame = FrameId::create("scanner").value();

  auto pipeline = InspectionPipeline::create(make_config());
  passed &= expect(static_cast<bool>(pipeline), "pipeline must be created from a valid config");

  if (pipeline) {
    // AC-001: identical surfaces must yield PASS with no defect regions.
    {
      const auto reference = make_patch(reference_frame, 6.0, 0.5, 0.0, 1.0);
      const auto scan = make_patch(scan_frame, 6.0, 0.5, 0.0, 1.0);
      auto result = pipeline.value().run(reference.surface.view(), scan.surface.view());
      passed &= expect(static_cast<bool>(result), "identical-surface run must complete");
      if (result) {
        passed &=
            expect(result.value().verdict == Verdict::pass, "identical surfaces must yield PASS");
        passed &= expect(result.value().regions.empty(),
                         "identical surfaces must produce no defect regions");
      }
    }

    // AC-003/004: a bump must yield FAIL with a single bump region.
    {
      const auto reference = make_patch(reference_frame, 6.0, 0.5, 0.0, 1.0);
      const auto bump = make_patch(scan_frame, 6.0, 0.5, 0.6, 2.0);
      auto result = pipeline.value().run(reference.surface.view(), bump.surface.view());
      passed &= expect(static_cast<bool>(result), "bump run must complete");
      if (result) {
        passed &= expect(result.value().verdict == Verdict::fail, "a bump must yield FAIL");
        passed &= expect(result.value().regions.size() == 1U &&
                             result.value().regions[0].type == pointcloud_ad::DefectType::bump,
                         "a bump must produce one bump region");
      }
    }

    // A grossly wrong initial pose must be rejected by the registration gate and yield
    // INDETERMINATE rather than proceeding to defect detection.
    {
      const auto reference = make_patch(reference_frame, 6.0, 0.5, 0.0, 1.0);
      // A scan displaced 100 mm has no correspondence within the search radius.
      std::vector<Vec3f> displaced_points;
      std::vector<Vec3f> displaced_normals;
      for (double x = -6.0; x <= 6.0 + 1.0e-9; x += 0.5) {
        for (double y = -6.0; y <= 6.0 + 1.0e-9; y += 0.5) {
          displaced_points.push_back({static_cast<float>(x), static_cast<float>(y), 100.0F});
          displaced_normals.push_back({0.0F, 0.0F, 1.0F});
        }
      }
      auto displaced =
          OwnedSurface::create(std::move(displaced_points), std::move(displaced_normals), {},
                               std::nullopt, LengthUnit::millimeter, scan_frame)
              .value();
      auto result = pipeline.value().run(reference.surface.view(), displaced.view());
      passed &= expect(static_cast<bool>(result), "bad-pose run must still complete");
      if (result) {
        passed &= expect(result.value().verdict == Verdict::indeterminate,
                         "a rejected registration must yield INDETERMINATE");
        passed &= expect(result.value().regions.empty(),
                         "a rejected registration must not run defect detection");
      }
    }
  }

  return passed ? 0 : 1;
}
