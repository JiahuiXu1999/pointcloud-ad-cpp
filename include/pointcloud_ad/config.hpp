#pragma once

#include <cmath>
#include <cstdint>
#include <exception>
#include <optional>
#include <pointcloud_ad/geometry.hpp>
#include <string>
#include <string_view>
#include <utility>

namespace pointcloud_ad {

struct InputConfig final {
  std::optional<LengthUnit> reference_unit;
  std::optional<LengthUnit> scan_unit;
  std::optional<FrameId> reference_frame;
  std::optional<FrameId> scan_frame;
};

struct PreprocessConfig final {
  std::optional<double> voxel_size_mm;
  std::optional<double> normal_radius_mm;
  std::optional<std::uint32_t> normal_min_neighbors;
  std::optional<double> boundary_radius_mm;
};

struct RegistrationConfig final {
  std::optional<std::uint32_t> max_iterations;
  std::optional<double> max_correspondence_distance_mm;
  std::optional<double> huber_delta_mm;
  std::optional<double> translation_epsilon_mm;
  std::optional<double> rotation_epsilon_rad;
  std::optional<double> residual_epsilon_mm;
};

struct RegistrationGateConfig final {
  std::optional<double> min_overlap_ratio;
  std::optional<double> max_inlier_rmse_mm;
  std::optional<std::uint32_t> min_valid_pairs;
  std::optional<double> max_translation_from_initial_mm;
  std::optional<double> max_rotation_from_initial_deg;
};

struct ComparisonConfig final {
  std::optional<double> max_search_distance_mm;
  std::optional<double> max_normal_angle_deg;
  std::optional<double> boundary_exclusion_mm;
  std::optional<double> min_valid_coverage_ratio;
};

struct DetectionConfig final {
  std::optional<double> positive_threshold_mm;
  std::optional<double> negative_threshold_mm;
  std::optional<double> cluster_tolerance_mm;
  std::optional<std::uint32_t> min_cluster_points;
  // The quantified profile error budget is required so detection thresholds can be proven to
  // exceed measurement uncertainty rather than relying on a production default.
  std::optional<double> measurement_error_budget_mm;
};

struct ExecutionConfig final {
  std::optional<bool> deterministic;
  std::optional<std::uint32_t> thread_count;
  std::optional<std::uint64_t> random_seed;
};

struct InspectionConfig final {
  std::string schema_version;
  std::string profile;
  InputConfig input;
  PreprocessConfig preprocess;
  RegistrationConfig registration;
  RegistrationGateConfig registration_gate;
  ComparisonConfig comparison;
  DetectionConfig detection;
  ExecutionConfig execution;
};

class ValidatedInspectionConfig;
namespace detail {
[[nodiscard]] Result<ValidatedInspectionConfig> validate_config_impl(InspectionConfig config);
}

struct ValidatedInputConfig final {
  LengthUnit reference_unit;
  LengthUnit scan_unit;
  FrameId reference_frame;
  FrameId scan_frame;
};

struct ValidatedPreprocessConfig final {
  double voxel_size_mm;
  double normal_radius_mm;
  std::uint32_t normal_min_neighbors;
  double boundary_radius_mm;
};

struct ValidatedRegistrationConfig final {
  std::uint32_t max_iterations;
  double max_correspondence_distance_mm;
  double huber_delta_mm;
  double translation_epsilon_mm;
  double rotation_epsilon_rad;
  double residual_epsilon_mm;
};

struct ValidatedRegistrationGateConfig final {
  double min_overlap_ratio;
  double max_inlier_rmse_mm;
  std::uint32_t min_valid_pairs;
  double max_translation_from_initial_mm;
  double max_rotation_from_initial_deg;
};

struct ValidatedComparisonConfig final {
  double max_search_distance_mm;
  double max_normal_angle_deg;
  double boundary_exclusion_mm;
  double min_valid_coverage_ratio;
};

struct ValidatedDetectionConfig final {
  double positive_threshold_mm;
  double negative_threshold_mm;
  double cluster_tolerance_mm;
  std::uint32_t min_cluster_points;
  double measurement_error_budget_mm;
};

struct ValidatedExecutionConfig final {
  bool deterministic;
  std::uint32_t thread_count;
  std::uint64_t random_seed;
};

class ValidatedInspectionConfig final {
public:
  [[nodiscard]] std::string_view schema_version() const noexcept {
    return schema_version_;
  }
  [[nodiscard]] std::string_view profile() const noexcept {
    return profile_;
  }
  [[nodiscard]] const ValidatedInputConfig& input() const noexcept {
    return input_;
  }
  [[nodiscard]] const ValidatedPreprocessConfig& preprocess() const noexcept {
    return preprocess_;
  }
  [[nodiscard]] const ValidatedRegistrationConfig& registration() const noexcept {
    return registration_;
  }
  [[nodiscard]] const ValidatedRegistrationGateConfig& registration_gate() const noexcept {
    return registration_gate_;
  }
  [[nodiscard]] const ValidatedComparisonConfig& comparison() const noexcept {
    return comparison_;
  }
  [[nodiscard]] const ValidatedDetectionConfig& detection() const noexcept {
    return detection_;
  }
  [[nodiscard]] const ValidatedExecutionConfig& execution() const noexcept {
    return execution_;
  }

private:
  friend Result<ValidatedInspectionConfig> validate_config(InspectionConfig) noexcept;
  friend Result<ValidatedInspectionConfig> detail::validate_config_impl(InspectionConfig);

  ValidatedInspectionConfig(std::string schema_version, std::string profile,
                            ValidatedInputConfig input, ValidatedPreprocessConfig preprocess,
                            ValidatedRegistrationConfig registration,
                            ValidatedRegistrationGateConfig registration_gate,
                            ValidatedComparisonConfig comparison,
                            ValidatedDetectionConfig detection, ValidatedExecutionConfig execution)
      : schema_version_(std::move(schema_version)), profile_(std::move(profile)),
        input_(std::move(input)), preprocess_(preprocess), registration_(registration),
        registration_gate_(registration_gate), comparison_(comparison), detection_(detection),
        execution_(execution) {}

  std::string schema_version_;
  std::string profile_;
  ValidatedInputConfig input_;
  ValidatedPreprocessConfig preprocess_;
  ValidatedRegistrationConfig registration_;
  ValidatedRegistrationGateConfig registration_gate_;
  ValidatedComparisonConfig comparison_;
  ValidatedDetectionConfig detection_;
  ValidatedExecutionConfig execution_;
};

namespace detail {

[[nodiscard]] inline Error config_error(std::string field, std::string reason) {
  return Error{ErrorCode::invalid_argument,
               PipelineStage::validate,
               "invalid inspection configuration",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

[[nodiscard]] inline bool positive_finite(double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] inline bool nonnegative_finite(double value) noexcept {
  return std::isfinite(value) && value >= 0.0;
}

[[nodiscard]] inline Result<ValidatedInspectionConfig>
validate_config_impl(InspectionConfig config) {
  const auto missing = [](std::string field) {
    return Result<ValidatedInspectionConfig>::failure(
        config_error(std::move(field), "is required"));
  };
  const auto invalid = [](std::string field, std::string rule) {
    return Result<ValidatedInspectionConfig>::failure(
        config_error(std::move(field), std::move(rule)));
  };

  if (config.schema_version.empty()) {
    return missing("schema_version");
  }
  if (config.schema_version != "1.0") {
    return invalid("schema_version", "must equal supported version 1.0");
  }
  if (config.profile.empty()) {
    return missing("profile");
  }
  if (!is_valid_utf8(config.profile)) {
    return invalid("profile", "must be valid UTF-8");
  }

  if (!config.input.reference_unit) {
    return missing("input.reference_unit");
  }
  if (!config.input.scan_unit) {
    return missing("input.scan_unit");
  }
  if (!millimeters_per_unit(*config.input.reference_unit)) {
    return invalid("input.reference_unit", "enumerator is not supported");
  }
  if (!millimeters_per_unit(*config.input.scan_unit)) {
    return invalid("input.scan_unit", "enumerator is not supported");
  }
  if (!config.input.reference_frame) {
    return missing("input.reference_frame");
  }
  if (!config.input.scan_frame) {
    return missing("input.scan_frame");
  }

  if (!config.preprocess.voxel_size_mm) {
    return missing("preprocess.voxel_size_mm");
  }
  if (!positive_finite(*config.preprocess.voxel_size_mm)) {
    return invalid("preprocess.voxel_size_mm", "must be finite and positive");
  }
  if (!config.preprocess.normal_radius_mm) {
    return missing("preprocess.normal_radius_mm");
  }
  if (!positive_finite(*config.preprocess.normal_radius_mm)) {
    return invalid("preprocess.normal_radius_mm", "must be finite and positive");
  }
  if (!config.preprocess.normal_min_neighbors) {
    return missing("preprocess.normal_min_neighbors");
  }
  if (*config.preprocess.normal_min_neighbors < 3U) {
    return invalid("preprocess.normal_min_neighbors", "must be at least 3");
  }
  if (!config.preprocess.boundary_radius_mm) {
    return missing("preprocess.boundary_radius_mm");
  }
  if (!positive_finite(*config.preprocess.boundary_radius_mm)) {
    return invalid("preprocess.boundary_radius_mm", "must be finite and positive");
  }

  if (!config.registration.max_iterations) {
    return missing("registration.max_iterations");
  }
  if (*config.registration.max_iterations < 1U || *config.registration.max_iterations > 1000U) {
    return invalid("registration.max_iterations", "must be in [1, 1000]");
  }
  if (!config.registration.max_correspondence_distance_mm) {
    return missing("registration.max_correspondence_distance_mm");
  }
  if (!positive_finite(*config.registration.max_correspondence_distance_mm)) {
    return invalid("registration.max_correspondence_distance_mm", "must be finite and positive");
  }
  if (!config.registration.huber_delta_mm) {
    return missing("registration.huber_delta_mm");
  }
  if (!positive_finite(*config.registration.huber_delta_mm)) {
    return invalid("registration.huber_delta_mm", "must be finite and positive");
  }
  if (!config.registration.translation_epsilon_mm) {
    return missing("registration.translation_epsilon_mm");
  }
  if (!positive_finite(*config.registration.translation_epsilon_mm)) {
    return invalid("registration.translation_epsilon_mm", "must be finite and positive");
  }
  if (!config.registration.rotation_epsilon_rad) {
    return missing("registration.rotation_epsilon_rad");
  }
  if (!positive_finite(*config.registration.rotation_epsilon_rad)) {
    return invalid("registration.rotation_epsilon_rad", "must be finite and positive");
  }
  if (!config.registration.residual_epsilon_mm) {
    return missing("registration.residual_epsilon_mm");
  }
  if (!positive_finite(*config.registration.residual_epsilon_mm)) {
    return invalid("registration.residual_epsilon_mm", "must be finite and positive");
  }

  if (!config.registration_gate.min_overlap_ratio) {
    return missing("registration_gate.min_overlap_ratio");
  }
  if (!std::isfinite(*config.registration_gate.min_overlap_ratio) ||
      *config.registration_gate.min_overlap_ratio < 0.0 ||
      *config.registration_gate.min_overlap_ratio > 1.0) {
    return invalid("registration_gate.min_overlap_ratio", "must be in [0, 1]");
  }
  if (!config.registration_gate.max_inlier_rmse_mm) {
    return missing("registration_gate.max_inlier_rmse_mm");
  }
  if (!positive_finite(*config.registration_gate.max_inlier_rmse_mm)) {
    return invalid("registration_gate.max_inlier_rmse_mm", "must be finite and positive");
  }
  if (!config.registration_gate.min_valid_pairs) {
    return missing("registration_gate.min_valid_pairs");
  }
  if (*config.registration_gate.min_valid_pairs == 0U) {
    return invalid("registration_gate.min_valid_pairs", "must be positive");
  }
  if (!config.registration_gate.max_translation_from_initial_mm) {
    return missing("registration_gate.max_translation_from_initial_mm");
  }
  if (!positive_finite(*config.registration_gate.max_translation_from_initial_mm)) {
    return invalid("registration_gate.max_translation_from_initial_mm",
                   "must be finite and positive");
  }
  if (!config.registration_gate.max_rotation_from_initial_deg) {
    return missing("registration_gate.max_rotation_from_initial_deg");
  }
  if (!positive_finite(*config.registration_gate.max_rotation_from_initial_deg) ||
      *config.registration_gate.max_rotation_from_initial_deg > 180.0) {
    return invalid("registration_gate.max_rotation_from_initial_deg", "must be in (0, 180]");
  }

  if (!config.comparison.max_search_distance_mm) {
    return missing("comparison.max_search_distance_mm");
  }
  if (!positive_finite(*config.comparison.max_search_distance_mm)) {
    return invalid("comparison.max_search_distance_mm", "must be finite and positive");
  }
  if (!config.comparison.max_normal_angle_deg) {
    return missing("comparison.max_normal_angle_deg");
  }
  if (!std::isfinite(*config.comparison.max_normal_angle_deg) ||
      *config.comparison.max_normal_angle_deg < 0.0 ||
      *config.comparison.max_normal_angle_deg > 90.0) {
    return invalid("comparison.max_normal_angle_deg", "must be in [0, 90]");
  }
  if (!config.comparison.boundary_exclusion_mm) {
    return missing("comparison.boundary_exclusion_mm");
  }
  if (!nonnegative_finite(*config.comparison.boundary_exclusion_mm)) {
    return invalid("comparison.boundary_exclusion_mm", "must be finite and non-negative");
  }
  if (!config.comparison.min_valid_coverage_ratio) {
    return missing("comparison.min_valid_coverage_ratio");
  }
  if (!std::isfinite(*config.comparison.min_valid_coverage_ratio) ||
      *config.comparison.min_valid_coverage_ratio < 0.0 ||
      *config.comparison.min_valid_coverage_ratio > 1.0) {
    return invalid("comparison.min_valid_coverage_ratio", "must be in [0, 1]");
  }

  if (!config.detection.positive_threshold_mm) {
    return missing("detection.positive_threshold_mm");
  }
  if (!positive_finite(*config.detection.positive_threshold_mm)) {
    return invalid("detection.positive_threshold_mm", "must be finite and positive");
  }
  if (!config.detection.negative_threshold_mm) {
    return missing("detection.negative_threshold_mm");
  }
  if (!std::isfinite(*config.detection.negative_threshold_mm) ||
      *config.detection.negative_threshold_mm >= 0.0) {
    return invalid("detection.negative_threshold_mm", "must be finite and negative");
  }
  if (!config.detection.cluster_tolerance_mm) {
    return missing("detection.cluster_tolerance_mm");
  }
  if (!positive_finite(*config.detection.cluster_tolerance_mm)) {
    return invalid("detection.cluster_tolerance_mm", "must be finite and positive");
  }
  if (!config.detection.min_cluster_points) {
    return missing("detection.min_cluster_points");
  }
  if (*config.detection.min_cluster_points == 0U) {
    return invalid("detection.min_cluster_points", "must be positive");
  }
  if (!config.detection.measurement_error_budget_mm) {
    return missing("detection.measurement_error_budget_mm");
  }
  if (!nonnegative_finite(*config.detection.measurement_error_budget_mm)) {
    return invalid("detection.measurement_error_budget_mm", "must be finite and non-negative");
  }
  if (*config.detection.positive_threshold_mm <= *config.detection.measurement_error_budget_mm) {
    return invalid("detection.positive_threshold_mm", "must exceed measurement error budget");
  }
  if (std::abs(*config.detection.negative_threshold_mm) <=
      *config.detection.measurement_error_budget_mm) {
    return invalid("detection.negative_threshold_mm",
                   "magnitude must exceed measurement error budget");
  }

  if (!config.execution.deterministic) {
    return missing("execution.deterministic");
  }
  if (!config.execution.thread_count) {
    return missing("execution.thread_count");
  }
  if (*config.execution.thread_count == 0U) {
    return invalid("execution.thread_count", "must be at least 1");
  }
  if (!config.execution.random_seed) {
    return missing("execution.random_seed");
  }

  return Result<ValidatedInspectionConfig>::success(ValidatedInspectionConfig{
      std::move(config.schema_version), std::move(config.profile),
      ValidatedInputConfig{*config.input.reference_unit, *config.input.scan_unit,
                           std::move(*config.input.reference_frame),
                           std::move(*config.input.scan_frame)},
      ValidatedPreprocessConfig{
          *config.preprocess.voxel_size_mm, *config.preprocess.normal_radius_mm,
          *config.preprocess.normal_min_neighbors, *config.preprocess.boundary_radius_mm},
      ValidatedRegistrationConfig{
          *config.registration.max_iterations, *config.registration.max_correspondence_distance_mm,
          *config.registration.huber_delta_mm, *config.registration.translation_epsilon_mm,
          *config.registration.rotation_epsilon_rad, *config.registration.residual_epsilon_mm},
      ValidatedRegistrationGateConfig{*config.registration_gate.min_overlap_ratio,
                                      *config.registration_gate.max_inlier_rmse_mm,
                                      *config.registration_gate.min_valid_pairs,
                                      *config.registration_gate.max_translation_from_initial_mm,
                                      *config.registration_gate.max_rotation_from_initial_deg},
      ValidatedComparisonConfig{
          *config.comparison.max_search_distance_mm, *config.comparison.max_normal_angle_deg,
          *config.comparison.boundary_exclusion_mm, *config.comparison.min_valid_coverage_ratio},
      ValidatedDetectionConfig{
          *config.detection.positive_threshold_mm, *config.detection.negative_threshold_mm,
          *config.detection.cluster_tolerance_mm, *config.detection.min_cluster_points,
          *config.detection.measurement_error_budget_mm},
      ValidatedExecutionConfig{*config.execution.deterministic, *config.execution.thread_count,
                               *config.execution.random_seed}});
}

} // namespace detail

[[nodiscard]] inline Result<ValidatedInspectionConfig>
validate_config(InspectionConfig config) noexcept {
  try {
    return detail::validate_config_impl(std::move(config));
  } catch (const std::exception& exception) {
    return Result<ValidatedInspectionConfig>::failure(
        Error{ErrorCode::internal_error,
              PipelineStage::validate,
              "unexpected exception while validating configuration",
              {{"exception", exception.what()}}});
  } catch (...) {
    return Result<ValidatedInspectionConfig>::failure(
        Error{ErrorCode::internal_error,
              PipelineStage::validate,
              "unknown exception while validating configuration",
              {}});
  }
}

} // namespace pointcloud_ad
