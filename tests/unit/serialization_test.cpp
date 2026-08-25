#include "json_serialization.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/inspection_result.hpp>
#include <string_view>
#include <utility>

namespace {

using pointcloud_ad::FrameId;
using pointcloud_ad::InspectionConfig;
using pointcloud_ad::InspectionResult;
using pointcloud_ad::LengthUnit;
using pointcloud_ad::Verdict;
using pointcloud_ad::serialization::parse_config;
using pointcloud_ad::serialization::serialize_config;
using pointcloud_ad::serialization::serialize_result;

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

  // Config round-trip preserves the required fields.
  {
    const auto config = make_config();
    auto serialized = serialize_config(config);
    passed &= expect(static_cast<bool>(serialized), "config must serialize");
    if (serialized) {
      auto parsed = parse_config(serialized.value());
      passed &= expect(static_cast<bool>(parsed), "serialized config must parse");
      if (parsed) {
        passed &= expect(parsed.value().schema_version == "1.0", "schema_version must round-trip");
        passed &= expect(parsed.value().profile == "synthetic_demo", "profile must round-trip");
        passed &= expect(parsed.value().input.reference_frame &&
                             parsed.value().input.reference_frame->value() == "fixture",
                         "reference frame must round-trip");
        passed &= expect(parsed.value().registration.max_iterations == 60U,
                         "max_iterations must round-trip");
        passed &=
            expect(parsed.value().execution.random_seed == 5489ULL, "random_seed must round-trip");
      }
    }
  }

  // Result serialization emits lowercase enums and null for non-finite numbers.
  {
    InspectionResult result;
    result.verdict = Verdict::fail;
    result.provenance.schema_version = "1.0";
    result.provenance.run_id = "run-1";
    result.provenance.profile = "synthetic_demo";
    result.provenance.timestamp_utc = "2026-08-25T00:00:00Z";
    result.deviations.mean_signed_mm = std::numeric_limits<double>::quiet_NaN();

    pointcloud_ad::DefectRegion region;
    region.id = 0U;
    region.type = pointcloud_ad::DefectType::bump;
    region.point_count = 42U;
    region.max_abs_mm = 0.6;
    region.severity = pointcloud_ad::Severity::reject;
    result.regions.push_back(region);

    auto serialized = serialize_result(result);
    passed &= expect(static_cast<bool>(serialized), "result must serialize");
    if (serialized) {
      const auto document = nlohmann::json::parse(serialized.value());
      passed &= expect(document["verdict"].get<std::string>() == "fail",
                       "verdict must serialize as lowercase");
      passed &= expect(document["regions"][0]["type"].get<std::string>() == "bump",
                       "region type must serialize as lowercase");
      passed &= expect(document["regions"][0]["severity"].get<std::string>() == "reject",
                       "severity must serialize as lowercase");
      passed &= expect(document["deviations"]["mean_signed_mm"].is_null(),
                       "non-finite numbers must serialize as null");
    }
  }

  return passed ? 0 : 1;
}
