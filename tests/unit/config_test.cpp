#include <iostream>
#include <limits>
#include <pointcloud_ad/config.hpp>
#include <string_view>
#include <utility>

namespace {

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

pointcloud_ad::InspectionConfig valid_config() {
  using pointcloud_ad::FrameId;
  using pointcloud_ad::LengthUnit;

  auto reference = FrameId::create("fixture");
  auto scan = FrameId::create("scanner");
  pointcloud_ad::InspectionConfig config;
  config.schema_version = "1.0";
  config.profile = "synthetic_demo";
  config.input.reference_unit = LengthUnit::millimeter;
  config.input.scan_unit = LengthUnit::millimeter;
  config.input.reference_frame = std::move(reference).value();
  config.input.scan_frame = std::move(scan).value();
  config.preprocess = {0.20, 1.00, 12U, 0.80};
  config.registration = {60U, 1.00, 0.30, 0.0001, 0.00001, 0.00001};
  config.registration_gate = {0.70, 0.20, 500U, 5.0, 5.0};
  config.comparison = {0.80, 35.0, 0.60, 0.75};
  config.detection = {0.25, -0.25, 0.60, 20U, 0.20};
  config.execution = {true, 1U, 5489U};
  return config;
}

bool expect_invalid(pointcloud_ad::InspectionConfig config, std::string_view expected_field) {
  auto result = pointcloud_ad::validate_config(std::move(config));
  if (result) {
    std::cerr << "FAILED: expected invalid field " << expected_field << '\n';
    return false;
  }
  const auto field = result.error().context.find("field");
  return expect(result.error().code == pointcloud_ad::ErrorCode::invalid_argument &&
                    result.error().stage == pointcloud_ad::PipelineStage::validate &&
                    field != result.error().context.end() && field->second == expected_field,
                "invalid config must report a stable field-level error");
}

} // namespace

int main() {
  bool passed = true;

  auto valid = pointcloud_ad::validate_config(valid_config());
  passed &=
      expect(static_cast<bool>(valid), "the documented synthetic configuration must validate");
  if (valid) {
    passed &= expect(valid.value().schema_version() == "1.0" &&
                         valid.value().profile() == "synthetic_demo",
                     "validated metadata must be retained");
    passed &= expect(valid.value().input().scan_frame.value() == "scanner" &&
                         valid.value().registration().max_iterations == 60U,
                     "validated values must be strongly typed and immutable through accessors");
    passed &= expect(valid.value().execution().deterministic &&
                         valid.value().execution().random_seed == 5489U,
                     "determinism controls must be retained for provenance");
  }

  auto missing_schema = valid_config();
  missing_schema.schema_version.clear();
  passed &= expect_invalid(std::move(missing_schema), "schema_version");

  auto unknown_schema = valid_config();
  unknown_schema.schema_version = "2.0";
  passed &= expect_invalid(std::move(unknown_schema), "schema_version");

  auto missing_gate = valid_config();
  missing_gate.registration_gate.max_inlier_rmse_mm.reset();
  passed &= expect_invalid(std::move(missing_gate), "registration_gate.max_inlier_rmse_mm");

  auto bad_iterations = valid_config();
  bad_iterations.registration.max_iterations = 1001U;
  passed &= expect_invalid(std::move(bad_iterations), "registration.max_iterations");

  auto bad_angle = valid_config();
  bad_angle.comparison.max_normal_angle_deg = 90.1;
  passed &= expect_invalid(std::move(bad_angle), "comparison.max_normal_angle_deg");

  auto nan_distance = valid_config();
  nan_distance.comparison.max_search_distance_mm = std::numeric_limits<double>::quiet_NaN();
  passed &= expect_invalid(std::move(nan_distance), "comparison.max_search_distance_mm");

  auto wrong_positive_sign = valid_config();
  wrong_positive_sign.detection.positive_threshold_mm = 0.0;
  passed &= expect_invalid(std::move(wrong_positive_sign), "detection.positive_threshold_mm");

  auto wrong_negative_sign = valid_config();
  wrong_negative_sign.detection.negative_threshold_mm = 0.25;
  passed &= expect_invalid(std::move(wrong_negative_sign), "detection.negative_threshold_mm");

  auto threshold_inside_budget = valid_config();
  threshold_inside_budget.detection.positive_threshold_mm = 0.20;
  passed &= expect_invalid(std::move(threshold_inside_budget), "detection.positive_threshold_mm");

  auto zero_threads = valid_config();
  zero_threads.execution.thread_count = 0U;
  passed &= expect_invalid(std::move(zero_threads), "execution.thread_count");

  auto missing_seed = valid_config();
  missing_seed.execution.random_seed.reset();
  passed &= expect_invalid(std::move(missing_seed), "execution.random_seed");

  return passed ? 0 : 1;
}
