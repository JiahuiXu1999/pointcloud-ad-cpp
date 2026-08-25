#include "json_serialization.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace pointcloud_ad::serialization {
namespace {

using json = nlohmann::json;

constexpr std::string_view kSchemaVersion = "1.0";

[[nodiscard]] Error serialization_error(ErrorCode code, std::string message) {
  return Error{code, PipelineStage::serialization, std::move(message), {}};
}

// --- enum helpers -------------------------------------------------------------

[[nodiscard]] std::string unit_to_string(LengthUnit unit) {
  switch (unit) {
  case LengthUnit::millimeter:
    return "millimeter";
  case LengthUnit::meter:
    return "meter";
  case LengthUnit::micrometer:
    return "micrometer";
  }
  return "millimeter";
}

[[nodiscard]] Result<LengthUnit> unit_from_string(std::string_view value) {
  if (value == "millimeter") {
    return Result<LengthUnit>::success(LengthUnit::millimeter);
  }
  if (value == "meter") {
    return Result<LengthUnit>::success(LengthUnit::meter);
  }
  if (value == "micrometer") {
    return Result<LengthUnit>::success(LengthUnit::micrometer);
  }
  return Result<LengthUnit>::failure(
      serialization_error(ErrorCode::invalid_input, "unknown length unit"));
}

[[nodiscard]] std::string verdict_to_string(Verdict verdict) {
  switch (verdict) {
  case Verdict::pass:
    return "pass";
  case Verdict::fail:
    return "fail";
  case Verdict::indeterminate:
    return "indeterminate";
  }
  return "indeterminate";
}

[[nodiscard]] std::string defect_type_to_string(DefectType type) {
  switch (type) {
  case DefectType::dent:
    return "dent";
  case DefectType::bump:
    return "bump";
  case DefectType::missing_material:
    return "missing_material";
  }
  return "dent";
}

[[nodiscard]] std::string severity_to_string(Severity severity) {
  switch (severity) {
  case Severity::info:
    return "info";
  case Severity::warning:
    return "warning";
  case Severity::reject:
    return "reject";
  }
  return "info";
}

[[nodiscard]] std::string error_code_to_string(ErrorCode code) {
  switch (code) {
  case ErrorCode::invalid_argument:
    return "invalid_argument";
  case ErrorCode::invalid_input:
    return "invalid_input";
  case ErrorCode::unsupported_format:
    return "unsupported_format";
  case ErrorCode::io_error:
    return "io_error";
  case ErrorCode::registration_failed:
    return "registration_failed";
  case ErrorCode::insufficient_coverage:
    return "insufficient_coverage";
  case ErrorCode::serialization_failed:
    return "serialization_failed";
  case ErrorCode::internal_error:
    return "internal_error";
  }
  return "internal_error";
}

[[nodiscard]] std::string stage_to_string(PipelineStage stage) {
  switch (stage) {
  case PipelineStage::none:
    return "none";
  case PipelineStage::validate:
    return "validate";
  case PipelineStage::normalize:
    return "normalize";
  case PipelineStage::preprocess:
    return "preprocess";
  case PipelineStage::registration:
    return "registration";
  case PipelineStage::registration_gate:
    return "registration_gate";
  case PipelineStage::transform:
    return "transform";
  case PipelineStage::compare:
    return "compare";
  case PipelineStage::coverage_gate:
    return "coverage_gate";
  case PipelineStage::detect:
    return "detect";
  case PipelineStage::finalize:
    return "finalize";
  case PipelineStage::serialization:
    return "serialization";
  }
  return "none";
}

// Emits a finite number or null, never NaN/Infinity.
[[nodiscard]] json finite_number(double value) {
  return std::isfinite(value) ? json(value) : json(nullptr);
}

// Emits an optional finite number or null.
[[nodiscard]] json optional_number(const std::optional<double>& value) {
  return value ? finite_number(*value) : json(nullptr);
}

// --- configuration -----------------------------------------------------------

[[nodiscard]] json serialize_transform(const RigidTransform& transform) {
  json array = json::array();
  for (double element : transform.matrix()) {
    array.push_back(element);
  }
  return json{{"source_frame", std::string(transform.source_frame().value())},
              {"target_frame", std::string(transform.target_frame().value())},
              {"matrix", std::move(array)}};
}

[[nodiscard]] json config_to_json(const InspectionConfig& config) {
  json input;
  input["reference_unit"] =
      config.input.reference_unit ? unit_to_string(*config.input.reference_unit) : std::string("");
  input["scan_unit"] =
      config.input.scan_unit ? unit_to_string(*config.input.scan_unit) : std::string("");
  input["reference_frame"] = config.input.reference_frame
                                 ? std::string(config.input.reference_frame->value())
                                 : std::string("");
  input["scan_frame"] =
      config.input.scan_frame ? std::string(config.input.scan_frame->value()) : std::string("");

  json preprocess;
  preprocess["voxel_size_mm"] = optional_number(config.preprocess.voxel_size_mm);
  preprocess["normal_radius_mm"] = optional_number(config.preprocess.normal_radius_mm);
  preprocess["normal_min_neighbors"] = config.preprocess.normal_min_neighbors
                                           ? json(*config.preprocess.normal_min_neighbors)
                                           : json(nullptr);
  preprocess["boundary_radius_mm"] = optional_number(config.preprocess.boundary_radius_mm);

  json registration;
  registration["max_iterations"] = config.registration.max_iterations
                                       ? json(*config.registration.max_iterations)
                                       : json(nullptr);
  registration["max_correspondence_distance_mm"] =
      optional_number(config.registration.max_correspondence_distance_mm);
  registration["huber_delta_mm"] = optional_number(config.registration.huber_delta_mm);
  registration["translation_epsilon_mm"] =
      optional_number(config.registration.translation_epsilon_mm);
  registration["rotation_epsilon_rad"] = optional_number(config.registration.rotation_epsilon_rad);
  registration["residual_epsilon_mm"] = optional_number(config.registration.residual_epsilon_mm);

  json registration_gate;
  registration_gate["min_overlap_ratio"] =
      optional_number(config.registration_gate.min_overlap_ratio);
  registration_gate["max_inlier_rmse_mm"] =
      optional_number(config.registration_gate.max_inlier_rmse_mm);
  registration_gate["min_valid_pairs"] = config.registration_gate.min_valid_pairs
                                             ? json(*config.registration_gate.min_valid_pairs)
                                             : json(nullptr);
  registration_gate["max_translation_from_initial_mm"] =
      optional_number(config.registration_gate.max_translation_from_initial_mm);
  registration_gate["max_rotation_from_initial_deg"] =
      optional_number(config.registration_gate.max_rotation_from_initial_deg);

  json comparison;
  comparison["max_search_distance_mm"] = optional_number(config.comparison.max_search_distance_mm);
  comparison["max_normal_angle_deg"] = optional_number(config.comparison.max_normal_angle_deg);
  comparison["boundary_exclusion_mm"] = optional_number(config.comparison.boundary_exclusion_mm);
  comparison["min_valid_coverage_ratio"] =
      optional_number(config.comparison.min_valid_coverage_ratio);

  json detection;
  detection["positive_threshold_mm"] = optional_number(config.detection.positive_threshold_mm);
  detection["negative_threshold_mm"] = optional_number(config.detection.negative_threshold_mm);
  detection["cluster_tolerance_mm"] = optional_number(config.detection.cluster_tolerance_mm);
  detection["min_cluster_points"] = config.detection.min_cluster_points
                                        ? json(*config.detection.min_cluster_points)
                                        : json(nullptr);
  detection["measurement_error_budget_mm"] =
      optional_number(config.detection.measurement_error_budget_mm);

  json execution;
  execution["deterministic"] =
      config.execution.deterministic ? json(*config.execution.deterministic) : json(nullptr);
  execution["thread_count"] =
      config.execution.thread_count ? json(*config.execution.thread_count) : json(nullptr);
  execution["random_seed"] =
      config.execution.random_seed ? json(*config.execution.random_seed) : json(nullptr);

  return json{{"schema_version", config.schema_version},
              {"profile", config.profile},
              {"input", std::move(input)},
              {"preprocess", std::move(preprocess)},
              {"registration", std::move(registration)},
              {"registration_gate", std::move(registration_gate)},
              {"comparison", std::move(comparison)},
              {"detection", std::move(detection)},
              {"execution", std::move(execution)}};
}

[[nodiscard]] std::optional<double> optional_double(const json& object, const char* key) {
  if (!object.contains(key) || object[key].is_null()) {
    return std::nullopt;
  }
  return object[key].get<double>();
}

[[nodiscard]] std::optional<std::uint32_t> optional_uint(const json& object, const char* key) {
  if (!object.contains(key) || object[key].is_null()) {
    return std::nullopt;
  }
  return object[key].get<std::uint32_t>();
}

[[nodiscard]] std::optional<bool> optional_bool(const json& object, const char* key) {
  if (!object.contains(key) || object[key].is_null()) {
    return std::nullopt;
  }
  return object[key].get<bool>();
}

[[nodiscard]] Result<InspectionConfig> json_to_config(const json& document) {
  if (!document.is_object() || !document.contains("schema_version")) {
    return Result<InspectionConfig>::failure(
        serialization_error(ErrorCode::invalid_input, "missing schema_version"));
  }
  const std::string schema_version = document["schema_version"].get<std::string>();
  if (schema_version != kSchemaVersion) {
    return Result<InspectionConfig>::failure(
        serialization_error(ErrorCode::invalid_input, "unsupported schema_version"));
  }

  InspectionConfig config;
  config.schema_version = schema_version;
  config.profile = document.value("profile", std::string(""));

  const json input = document.value("input", json::object());
  if (input.contains("reference_unit") && !input["reference_unit"].is_null()) {
    auto unit = unit_from_string(input["reference_unit"].get<std::string>());
    if (!unit) {
      return Result<InspectionConfig>::failure(std::move(unit).error());
    }
    config.input.reference_unit = unit.value();
  }
  if (input.contains("scan_unit") && !input["scan_unit"].is_null()) {
    auto unit = unit_from_string(input["scan_unit"].get<std::string>());
    if (!unit) {
      return Result<InspectionConfig>::failure(std::move(unit).error());
    }
    config.input.scan_unit = unit.value();
  }
  if (input.contains("reference_frame") && !input["reference_frame"].is_null()) {
    auto frame = FrameId::create(input["reference_frame"].get<std::string>());
    if (!frame) {
      return Result<InspectionConfig>::failure(std::move(frame).error());
    }
    config.input.reference_frame = std::move(frame).value();
  }
  if (input.contains("scan_frame") && !input["scan_frame"].is_null()) {
    auto frame = FrameId::create(input["scan_frame"].get<std::string>());
    if (!frame) {
      return Result<InspectionConfig>::failure(std::move(frame).error());
    }
    config.input.scan_frame = std::move(frame).value();
  }

  const json preprocess = document.value("preprocess", json::object());
  config.preprocess.voxel_size_mm = optional_double(preprocess, "voxel_size_mm");
  config.preprocess.normal_radius_mm = optional_double(preprocess, "normal_radius_mm");
  config.preprocess.normal_min_neighbors = optional_uint(preprocess, "normal_min_neighbors");
  config.preprocess.boundary_radius_mm = optional_double(preprocess, "boundary_radius_mm");

  const json registration = document.value("registration", json::object());
  config.registration.max_iterations = optional_uint(registration, "max_iterations");
  config.registration.max_correspondence_distance_mm =
      optional_double(registration, "max_correspondence_distance_mm");
  config.registration.huber_delta_mm = optional_double(registration, "huber_delta_mm");
  config.registration.translation_epsilon_mm =
      optional_double(registration, "translation_epsilon_mm");
  config.registration.rotation_epsilon_rad = optional_double(registration, "rotation_epsilon_rad");
  config.registration.residual_epsilon_mm = optional_double(registration, "residual_epsilon_mm");

  const json registration_gate = document.value("registration_gate", json::object());
  config.registration_gate.min_overlap_ratio =
      optional_double(registration_gate, "min_overlap_ratio");
  config.registration_gate.max_inlier_rmse_mm =
      optional_double(registration_gate, "max_inlier_rmse_mm");
  config.registration_gate.min_valid_pairs = optional_uint(registration_gate, "min_valid_pairs");
  config.registration_gate.max_translation_from_initial_mm =
      optional_double(registration_gate, "max_translation_from_initial_mm");
  config.registration_gate.max_rotation_from_initial_deg =
      optional_double(registration_gate, "max_rotation_from_initial_deg");

  const json comparison = document.value("comparison", json::object());
  config.comparison.max_search_distance_mm = optional_double(comparison, "max_search_distance_mm");
  config.comparison.max_normal_angle_deg = optional_double(comparison, "max_normal_angle_deg");
  config.comparison.boundary_exclusion_mm = optional_double(comparison, "boundary_exclusion_mm");
  config.comparison.min_valid_coverage_ratio =
      optional_double(comparison, "min_valid_coverage_ratio");

  const json detection = document.value("detection", json::object());
  config.detection.positive_threshold_mm = optional_double(detection, "positive_threshold_mm");
  config.detection.negative_threshold_mm = optional_double(detection, "negative_threshold_mm");
  config.detection.cluster_tolerance_mm = optional_double(detection, "cluster_tolerance_mm");
  config.detection.min_cluster_points = optional_uint(detection, "min_cluster_points");
  config.detection.measurement_error_budget_mm =
      optional_double(detection, "measurement_error_budget_mm");

  const json execution = document.value("execution", json::object());
  config.execution.deterministic = optional_bool(execution, "deterministic");
  config.execution.thread_count = optional_uint(execution, "thread_count");
  config.execution.random_seed =
      execution.contains("random_seed") && !execution["random_seed"].is_null()
          ? std::optional<std::uint64_t>(execution["random_seed"].get<std::uint64_t>())
          : std::nullopt;

  return Result<InspectionConfig>::success(std::move(config));
}

// --- result ------------------------------------------------------------------

[[nodiscard]] json serialize_vec3(Vec3d value) {
  return json{{"x", value.x}, {"y", value.y}, {"z", value.z}};
}

[[nodiscard]] json serialize_result_json(const InspectionResult& result) {
  json registration;
  registration["initial_pose"] = result.registration.initial_pose
                                     ? serialize_transform(*result.registration.initial_pose)
                                     : json(nullptr);
  registration["final_pose"] = result.registration.final_pose
                                   ? serialize_transform(*result.registration.final_pose)
                                   : json(nullptr);
  registration["iterations"] = result.registration.iterations;
  registration["converged"] = result.registration.converged;
  registration["valid_pairs"] = result.registration.valid_pairs;
  registration["overlap_ratio"] = finite_number(result.registration.overlap_ratio);
  registration["inlier_rmse_mm"] = finite_number(result.registration.inlier_rmse_mm);
  registration["termination_reason"] = result.registration.termination_reason;

  json coverage;
  coverage["valid_count"] = result.coverage.valid_count;
  coverage["covered_count"] = result.coverage.covered_count;
  coverage["coverage_ratio"] = finite_number(result.coverage.coverage_ratio);
  coverage["input_invalid"] = result.coverage.input_invalid;
  coverage["no_neighbor"] = result.coverage.no_neighbor;
  coverage["scan_boundary"] = result.coverage.scan_boundary;

  json deviations;
  deviations["valid_count"] = result.deviations.valid_count;
  deviations["mean_signed_mm"] = finite_number(result.deviations.mean_signed_mm);
  deviations["rms_mm"] = finite_number(result.deviations.rms_mm);
  deviations["max_abs_mm"] = finite_number(result.deviations.max_abs_mm);
  deviations["p95_abs_mm"] = finite_number(result.deviations.p95_abs_mm);
  deviations["input_invalid"] = result.deviations.input_invalid;
  deviations["no_neighbor"] = result.deviations.no_neighbor;
  deviations["normal_missing"] = result.deviations.normal_missing;
  deviations["normal_mismatch"] = result.deviations.normal_mismatch;
  deviations["reference_boundary"] = result.deviations.reference_boundary;

  json regions = json::array();
  for (const auto& region : result.regions) {
    regions.push_back(json{{"id", region.id},
                           {"type", defect_type_to_string(region.type)},
                           {"point_count", region.point_count},
                           {"centroid", serialize_vec3(region.centroid)},
                           {"aabb_min", serialize_vec3(region.aabb_min)},
                           {"aabb_max", serialize_vec3(region.aabb_max)},
                           {"max_abs_mm", finite_number(region.max_abs_mm)},
                           {"mean_mm", finite_number(region.mean_mm)},
                           {"rms_mm", finite_number(region.rms_mm)},
                           {"p95_abs_mm", finite_number(region.p95_abs_mm)},
                           {"estimated_area_mm2", optional_number(region.estimated_area_mm2)},
                           {"area_is_approximate", region.area_is_approximate},
                           {"severity", severity_to_string(region.severity)}});
  }

  json timings = json::array();
  for (const auto& timing : result.timings) {
    timings.push_back(
        json{{"stage", stage_to_string(timing.stage)}, {"duration_us", timing.duration_us}});
  }

  json provenance;
  provenance["schema_version"] = result.provenance.schema_version;
  provenance["run_id"] = result.provenance.run_id;
  provenance["profile"] = result.provenance.profile;
  provenance["generator_version"] = result.provenance.generator_version;
  provenance["timestamp_utc"] = result.provenance.timestamp_utc;
  provenance["deterministic"] = result.provenance.deterministic;
  provenance["thread_count"] = result.provenance.thread_count;
  provenance["random_seed"] = result.provenance.random_seed;

  json diagnostics = json::array();
  for (const auto& diagnostic : result.diagnostics) {
    diagnostics.push_back(json{{"code", error_code_to_string(diagnostic.code)},
                               {"stage", stage_to_string(diagnostic.stage)},
                               {"message", diagnostic.message},
                               {"context", diagnostic.context}});
  }

  return json{{"schema_version", result.provenance.schema_version.empty()
                                     ? std::string(kSchemaVersion)
                                     : result.provenance.schema_version},
              {"verdict", verdict_to_string(result.verdict)},
              {"registration", std::move(registration)},
              {"coverage", std::move(coverage)},
              {"deviations", std::move(deviations)},
              {"regions", std::move(regions)},
              {"timings", std::move(timings)},
              {"provenance", std::move(provenance)},
              {"diagnostics", std::move(diagnostics)}};
}

} // namespace

Result<std::string> serialize_config(const InspectionConfig& config) noexcept {
  try {
    return Result<std::string>::success(config_to_json(config).dump());
  } catch (const std::exception& exception) {
    return Result<std::string>::failure(
        serialization_error(ErrorCode::internal_error, exception.what()));
  } catch (...) {
    return Result<std::string>::failure(
        serialization_error(ErrorCode::internal_error, "unknown exception"));
  }
}

Result<InspectionConfig> parse_config(std::string_view json_text) noexcept {
  try {
    return json_to_config(json::parse(json_text));
  } catch (const std::exception& exception) {
    return Result<InspectionConfig>::failure(
        serialization_error(ErrorCode::invalid_input, exception.what()));
  } catch (...) {
    return Result<InspectionConfig>::failure(
        serialization_error(ErrorCode::invalid_input, "unknown exception"));
  }
}

Result<std::string> serialize_result(const InspectionResult& result) noexcept {
  try {
    return Result<std::string>::success(serialize_result_json(result).dump());
  } catch (const std::exception& exception) {
    return Result<std::string>::failure(
        serialization_error(ErrorCode::internal_error, exception.what()));
  } catch (...) {
    return Result<std::string>::failure(
        serialization_error(ErrorCode::internal_error, "unknown exception"));
  }
}

} // namespace pointcloud_ad::serialization
