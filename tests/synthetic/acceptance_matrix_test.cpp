#include "synthetic_scenes.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/inspection_pipeline.hpp>
#include <pointcloud_ad/inspection_result.hpp>
#include <pointcloud_ad/status.hpp>
#include <pointcloud_ad/surface.hpp>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using pointcloud_ad::ErrorCode;
using pointcloud_ad::FrameId;
using pointcloud_ad::InspectionConfig;
using pointcloud_ad::InspectionPipeline;
using pointcloud_ad::InspectionResult;
using pointcloud_ad::LengthUnit;
using pointcloud_ad::OwnedSurface;
using pointcloud_ad::Result;
using pointcloud_ad::RigidTransform;
using pointcloud_ad::SurfaceView;
using pointcloud_ad::Vec3d;
using pointcloud_ad::Vec3f;
using pointcloud_ad::Verdict;
using namespace pointcloud_ad::synthetic;

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
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
  config.comparison.max_search_distance_mm = 1.5;
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

[[nodiscard]] double rotation_angle_deg(const RigidTransform& left, const RigidTransform& right) {
  const auto& left_matrix = left.matrix();
  const auto& right_matrix = right.matrix();
  // trace(R_left^T * R_right) = sum_{i,k} R_left[k][i] * R_right[k][i].
  double trace = 0.0;
  for (std::size_t k = 0; k < 3; ++k) {
    for (std::size_t i = 0; i < 3; ++i) {
      trace += left_matrix[k * 4 + i] * right_matrix[k * 4 + i];
    }
  }
  const double cosine = std::clamp((trace - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cosine) * 180.0 / 3.14159265358979323846;
}

[[nodiscard]] double translation_distance_mm(const RigidTransform& left,
                                             const RigidTransform& right) {
  const auto& left_matrix = left.matrix();
  const auto& right_matrix = right.matrix();
  const double dx = left_matrix[3] - right_matrix[3];
  const double dy = left_matrix[7] - right_matrix[7];
  const double dz = left_matrix[11] - right_matrix[11];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

[[nodiscard]] std::size_t block_point_count(SurfaceView reference, Vec3d center,
                                            double radius) {
  std::size_t count = 0;
  for (const auto point : reference.points()) {
    const double dx = static_cast<double>(point.x) - center.x;
    const double dy = static_cast<double>(point.y) - center.y;
    if (dx * dx + dy * dy <= radius * radius) {
      ++count;
    }
  }
  return count;
}

} // namespace

int main() {
  bool passed = true;

  auto pipeline = InspectionPipeline::create(make_config());
  passed &= expect(static_cast<bool>(pipeline), "pipeline must be created from a valid config");
  if (!pipeline) {
    return 1;
  }

  // AC-001: identical surfaces must PASS with no regions and no measurable deviation.
  {
    const auto scene = make_identical();
    auto result = pipeline.value().run(scene.reference.view(), scene.scan.view());
    passed &= expect(static_cast<bool>(result), "AC-001 run must complete");
    if (result) {
      passed &= expect(result.value().verdict == Verdict::pass, "AC-001 must PASS");
      passed &= expect(result.value().deviations.max_abs_mm <= 1.0e-5,
                       "AC-001 max_abs_mm must be at most 1e-5");
      passed &= expect(result.value().regions.empty(), "AC-001 must report no regions");
    }
  }

  // AC-002: a known rigid scan->reference pose must be recovered and PASS.
  {
    const auto scene = make_rigid();
    auto result = pipeline.value().run(scene.reference.view(), scene.scan.view());
    passed &= expect(static_cast<bool>(result), "AC-002 run must complete");
    if (!result) {
      std::cerr << "  AC-002 error: " << result.error().message << '\n';
      for (const auto& [key, value] : result.error().context) {
        std::cerr << "    " << key << ": " << value << '\n';
      }
    }
    if (result) {
      passed &= expect(result.value().verdict == Verdict::pass, "AC-002 must PASS");
      if (result.value().registration.final_pose && scene.truth.ground_truth_pose) {
        const auto& final_pose = *result.value().registration.final_pose;
        const auto& truth_pose = *scene.truth.ground_truth_pose;
        passed &= expect(translation_distance_mm(final_pose, truth_pose) <= 0.02,
                         "AC-002 translation error must be at most 0.02 mm");
        passed &= expect(rotation_angle_deg(final_pose, truth_pose) <= 0.02,
                         "AC-002 rotation error must be at most 0.02 degrees");
        if (rotation_angle_deg(final_pose, truth_pose) > 0.02) {
          std::cerr << "  AC-002 rotation error=" << rotation_angle_deg(final_pose, truth_pose)
                    << '\n';
        }
      } else {
        passed &= expect(false, "AC-002 final pose must be present");
      }
    }
  }

  // AC-003: a planar bump must classify as a single bump with a bounded depth error.
  {
    const auto scene = make_bump();
    auto result = pipeline.value().run(scene.reference.view(), scene.scan.view());
    passed &= expect(static_cast<bool>(result), "AC-003 run must complete");
    if (result) {
      passed &= expect(result.value().verdict == Verdict::fail, "AC-003 must FAIL");
      passed &= expect(result.value().regions.size() == 1U &&
                           result.value().regions[0].type == pointcloud_ad::DefectType::bump,
                       "AC-003 must yield a single bump region");
      if (!result.value().regions.empty()) {
        const double tolerance = std::max(0.02, std::abs(scene.truth.injected_depth_mm) * 0.05);
        passed &= expect(std::abs(result.value().regions[0].max_abs_mm -
                                  scene.truth.injected_depth_mm) <= tolerance,
                         "AC-003 bump depth error must be bounded");
        if (std::abs(result.value().regions[0].max_abs_mm - scene.truth.injected_depth_mm) >
            tolerance) {
          std::cerr << "  AC-003 max_abs=" << result.value().regions[0].max_abs_mm
                    << " injected=" << scene.truth.injected_depth_mm << '\n';
        }
      }
    }
  }

  // AC-004: a planar dent must classify as a single dent with correct sign, centre, and depth.
  {
    const auto scene = make_dent();
    auto result = pipeline.value().run(scene.reference.view(), scene.scan.view());
    passed &= expect(static_cast<bool>(result), "AC-004 run must complete");
    if (result) {
      passed &= expect(result.value().verdict == Verdict::fail, "AC-004 must FAIL");
      passed &= expect(result.value().regions.size() == 1U &&
                           result.value().regions[0].type == pointcloud_ad::DefectType::dent,
                       "AC-004 must yield a single dent region");
      if (!result.value().regions.empty()) {
        const auto& region = result.value().regions[0];
        passed &= expect(region.mean_mm < 0.0, "AC-004 dent must have a negative mean deviation");
        passed &= expect(std::abs(region.centroid.x) <= 0.3 && std::abs(region.centroid.y) <= 0.3,
                         "AC-004 dent centre must match the injected position");
        const double tolerance = std::max(0.02, std::abs(scene.truth.injected_depth_mm) * 0.05);
        passed &= expect(std::abs(region.max_abs_mm - std::abs(scene.truth.injected_depth_mm)) <=
                             tolerance,
                         "AC-004 dent depth error must be bounded");
        if (std::abs(region.max_abs_mm - std::abs(scene.truth.injected_depth_mm)) > tolerance) {
          std::cerr << "  AC-004 max_abs=" << region.max_abs_mm
                    << " injected=" << std::abs(scene.truth.injected_depth_mm)
                    << " mean=" << region.mean_mm << '\n';
        }
      }
    }
  }

  // AC-005: a removed scan block must cluster with at least 95% recall.
  {
    const auto scene = make_missing();
    auto result = pipeline.value().run(scene.reference.view(), scene.scan.view());
    passed &= expect(static_cast<bool>(result), "AC-005 run must complete");
    if (result) {
      std::size_t missing_points = 0;
      for (const auto& region : result.value().regions) {
        if (region.type == pointcloud_ad::DefectType::missing_material) {
          missing_points += region.point_count;
        }
      }
      // Recall is measured against the guaranteed-uncovered core of the removed block; the outer
      // rim stays covered by the scan and must not dilute the recall denominator.
      const std::size_t total =
          block_point_count(scene.reference.view(), *scene.truth.missing_center_mm,
                            scene.truth.missing_core_radius_mm);
      passed &= expect(total > 0U, "AC-005 the missing block must contain reference points");
      const double recall =
          total == 0U ? 0.0 : static_cast<double>(missing_points) / static_cast<double>(total);
      passed &= expect(recall >= 0.95, "AC-005 missing-material recall must be at least 0.95");
      if (recall < 0.95) {
        std::cerr << "  AC-005 recall=" << recall << " covered=" << missing_points
                  << " total=" << total << '\n';
      }
    }
  }

  // AC-006: a partial scan below the coverage gate must never PASS.
  {
    const auto scene = make_dropout();
    auto result = pipeline.value().run(scene.reference.view(), scene.scan.view());
    passed &= expect(static_cast<bool>(result), "AC-006 run must complete");
    if (result) {
      passed &= expect(result.value().verdict != Verdict::pass,
                       "AC-006 a dropped-out scan must not PASS");
    }
  }

  // AC-007: a grossly wrong initial pose must fail registration without defect detection.
  {
    const auto scene = make_bad_pose();
    auto result = pipeline.value().run(scene.reference.view(), scene.scan.view());
    passed &= expect(static_cast<bool>(result), "AC-007 run must complete");
    if (result) {
      passed &= expect(result.value().verdict == Verdict::indeterminate,
                       "AC-007 a bad pose must yield INDETERMINATE");
      passed &= expect(result.value().regions.empty(),
                       "AC-007 a rejected registration must not run defect detection");
    }
  }

  // AC-008: flipped scan normals make signed deviation unreliable; must never PASS.
  {
    const auto scene = make_normal_orientation();
    auto result = pipeline.value().run(scene.reference.view(), scene.scan.view());
    passed &= expect(static_cast<bool>(result), "AC-008 run must complete");
    if (result) {
      passed &= expect(result.value().verdict != Verdict::pass,
                       "AC-008 unproven normal orientation must not PASS");
      passed &= expect(result.value().deviations.valid_count == 0U,
                       "AC-008 flipped normals must leave no trustworthy deviation samples");
    }
  }

  // AC-009: a boundary-adjacent anomaly must be suppressed by the boundary mask.
  {
    const auto scene = make_boundary_mask();
    auto result = pipeline.value().run(scene.reference.view(), scene.scan.view());
    passed &= expect(static_cast<bool>(result), "AC-009 run must complete");
    if (result) {
      passed &= expect(result.value().regions.empty(),
                       "AC-009 boundary-adjacent anomalies must not create false regions");
    }
  }

  // AC-010: NaN/Inf input must fail with a stable invalid_input error and never crash.
  {
    const auto frame = FrameId::create("scanner").value();
    std::vector<Vec3f> points{{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}};
    points.push_back({std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F});
    auto surface = OwnedSurface::create(std::move(points), {}, {}, std::nullopt,
                                        LengthUnit::millimeter, frame);
    passed &= expect(!surface, "AC-010 a NaN valid point must fail surface validation");
    if (!surface) {
      passed &= expect(surface.error().code == ErrorCode::invalid_input,
                       "AC-010 must report invalid_input");
    }
  }

  // AC-011: identical runs on a fixed configuration must be deterministic across 10 runs.
  {
    const auto scene = make_identical();
    std::optional<Verdict> first_verdict;
    std::optional<std::size_t> first_regions;
    std::optional<double> first_max_abs;
    std::optional<double> first_rms;
    std::optional<double> first_coverage;
    for (int iteration = 0; iteration < 10; ++iteration) {
      auto result = pipeline.value().run(scene.reference.view(), scene.scan.view());
      if (!result) {
        passed &= expect(false, "AC-011 run must complete");
        break;
      }
      const auto& report = result.value();
      if (iteration == 0) {
        first_verdict = report.verdict;
        first_regions = report.regions.size();
        first_max_abs = report.deviations.max_abs_mm;
        first_rms = report.deviations.rms_mm;
        first_coverage = report.coverage.coverage_ratio;
        continue;
      }
      passed &= expect(report.verdict == *first_verdict,
                       "AC-011 verdict must be stable across runs");
      passed &= expect(report.regions.size() == *first_regions,
                       "AC-011 region count must be stable across runs");
      passed &= expect(report.deviations.max_abs_mm == *first_max_abs,
                       "AC-011 max_abs_mm must be stable across runs");
      passed &= expect(report.deviations.rms_mm == *first_rms,
                       "AC-011 rms_mm must be stable across runs");
      passed &= expect(report.coverage.coverage_ratio == *first_coverage,
                       "AC-011 coverage ratio must be stable across runs");
    }
  }

  // AC-012 (consumer_install) is covered by the Release -VerifyInstall consumer smoke test and is
  // intentionally not repeated here.

  return passed ? 0 : 1;
}
