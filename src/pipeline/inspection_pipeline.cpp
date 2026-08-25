#include "comparison_statistics.hpp"
#include "coverage_field.hpp"
#include "defect_cluster.hpp"
#include "defect_region.hpp"
#include "deviation_field.hpp"
#include "normal_boundary.hpp"
#include "pcl_registration_backend.hpp"
#include "registration_gate.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <exception>
#include <iomanip>
#include <optional>
#include <pointcloud_ad/inspection_pipeline.hpp>
#include <pointcloud_ad/normalization.hpp>
#include <pointcloud_ad/version.hpp>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_ad {

namespace {

constexpr std::array<double, 16> kIdentity{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                           0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};

[[nodiscard]] Error pipeline_error(ErrorCode code, std::string message) {
  return Error{code, PipelineStage::none, std::move(message), {}};
}

[[nodiscard]] std::string utc_timestamp() {
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
#ifdef _WIN32
  gmtime_s(&utc, &time);
#else
  gmtime_r(&time, &utc);
#endif
  std::ostringstream stream;
  stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

class StageClock final {
public:
  StageClock(std::vector<StageTiming>& timings, PipelineStage stage) : timings_(timings) {
    record_.stage = stage;
    start_ = std::chrono::steady_clock::now();
  }
  ~StageClock() {
    record_.duration_us =
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::steady_clock::now() - start_)
                                       .count());
    timings_.push_back(record_);
  }

private:
  std::vector<StageTiming>& timings_;
  StageTiming record_;
  std::chrono::steady_clock::time_point start_;
};

[[nodiscard]] RegistrationParameters
to_registration_parameters(const ValidatedRegistrationConfig& config) noexcept {
  RegistrationParameters parameters;
  parameters.max_iterations = config.max_iterations;
  parameters.max_correspondence_distance_mm = config.max_correspondence_distance_mm;
  parameters.huber_delta_mm = config.huber_delta_mm;
  parameters.translation_epsilon_mm = config.translation_epsilon_mm;
  parameters.rotation_epsilon_rad = config.rotation_epsilon_rad;
  parameters.residual_epsilon_mm = config.residual_epsilon_mm;
  return parameters;
}

[[nodiscard]] std::string termination_reason(RegistrationConvergence convergence) {
  switch (convergence) {
  case RegistrationConvergence::converged:
    return "converged";
  case RegistrationConvergence::not_converged:
    return "not_converged";
  case RegistrationConvergence::degenerate_input:
    return "degenerate_input";
  }
  return "unknown";
}

[[nodiscard]] Severity default_severity(DefectType type, double max_abs_mm,
                                        const ValidatedDetectionConfig& config) noexcept {
  if (type == DefectType::missing_material) {
    return Severity::reject;
  }
  const double threshold = std::max(config.positive_threshold_mm, -config.negative_threshold_mm);
  if (max_abs_mm >= 2.0 * threshold) {
    return Severity::reject;
  }
  return Severity::warning;
}

[[nodiscard]] DefectType to_public_type(detection::DefectType type) noexcept {
  switch (type) {
  case detection::DefectType::dent:
    return DefectType::dent;
  case detection::DefectType::bump:
    return DefectType::bump;
  case detection::DefectType::missing_material:
    return DefectType::missing_material;
  }
  return DefectType::dent;
}

[[nodiscard]] Result<InspectionResult> run_impl(const ValidatedInspectionConfig& config,
                                                SurfaceView reference, SurfaceView scan,
                                                const InspectionRequest& request) {
  InspectionResult result;
  result.provenance.schema_version = std::string(config.schema_version());
  result.provenance.run_id = request.run_id;
  result.provenance.profile = std::string(config.profile());
  result.provenance.generator_version = std::string(version_string());
  result.provenance.timestamp_utc = utc_timestamp();
  result.provenance.deterministic = config.execution().deterministic;
  result.provenance.thread_count = config.execution().thread_count;
  result.provenance.random_seed = config.execution().random_seed;

  // P01/P02: validate and normalize both surfaces to millimetres.
  std::optional<OwnedSurface> reference_mm;
  std::optional<OwnedSurface> scan_mm;
  {
    StageClock clock(result.timings, PipelineStage::normalize);
    auto normalized_reference = normalize_surface(reference, reference.frame(), std::nullopt);
    if (!normalized_reference) {
      return Result<InspectionResult>::failure(std::move(normalized_reference).error());
    }
    auto normalized_scan = normalize_surface(scan, scan.frame(), std::nullopt);
    if (!normalized_scan) {
      return Result<InspectionResult>::failure(std::move(normalized_scan).error());
    }
    reference_mm.emplace(std::move(normalized_reference).value());
    scan_mm.emplace(std::move(normalized_scan).value());
  }

  // P03: prepare normals and boundaries for both surfaces.
  std::optional<preprocess::NormalBoundaryResult> reference_prepared;
  std::optional<preprocess::NormalBoundaryResult> scan_prepared;
  {
    StageClock clock(result.timings, PipelineStage::preprocess);
    auto reference_result = preprocess::prepare_normals_and_boundaries(
        reference_mm->view(), config.preprocess().normal_radius_mm,
        config.preprocess().normal_min_neighbors, config.preprocess().boundary_radius_mm);
    if (!reference_result) {
      return Result<InspectionResult>::failure(std::move(reference_result).error());
    }
    auto scan_result = preprocess::prepare_normals_and_boundaries(
        scan_mm->view(), config.preprocess().normal_radius_mm,
        config.preprocess().normal_min_neighbors, config.preprocess().boundary_radius_mm);
    if (!scan_result) {
      return Result<InspectionResult>::failure(std::move(scan_result).error());
    }
    reference_prepared.emplace(std::move(reference_result).value());
    scan_prepared.emplace(std::move(scan_result).value());
  }

  // P04: register the scan to the reference with the initial pose (identity when absent).
  std::optional<RegistrationMetrics> metrics;
  {
    StageClock clock(result.timings, PipelineStage::registration);
    std::optional<RigidTransform> initial;
    if (request.initial_pose) {
      if (request.initial_pose->source_frame() != scan_prepared->surface().frame() ||
          request.initial_pose->target_frame() != reference_prepared->surface().frame()) {
        return Result<InspectionResult>::failure(pipeline_error(
            ErrorCode::invalid_input, "initial pose frame direction must match scan-to-reference"));
      }
      initial.emplace(*request.initial_pose);
    } else {
      auto identity = RigidTransform::create(kIdentity, scan_prepared->surface().frame(),
                                             reference_prepared->surface().frame());
      if (!identity) {
        return Result<InspectionResult>::failure(std::move(identity).error());
      }
      initial.emplace(std::move(identity).value());
      result.diagnostics.push_back(Diagnostic{ErrorCode::invalid_input,
                                              PipelineStage::registration,
                                              "no initial pose provided; using identity",
                                              {}});
    }
    result.registration.initial_pose = *initial;
    auto solved = backends::pcl_backend::align_point_to_plane(
        reference_prepared->surface(), scan_prepared->surface(), *initial,
        to_registration_parameters(config.registration()));
    if (!solved) {
      return Result<InspectionResult>::failure(std::move(solved).error());
    }
    metrics.emplace(std::move(solved).value());
  }

  result.registration.final_pose = metrics->final_transform();
  result.registration.iterations = metrics->iterations();
  result.registration.converged = metrics->convergence() == RegistrationConvergence::converged;
  result.registration.valid_pairs = metrics->valid_pairs();
  result.registration.overlap_ratio = metrics->fitness();
  result.registration.inlier_rmse_mm = metrics->inlier_rmse_mm();
  result.registration.termination_reason = termination_reason(metrics->convergence());

  // P05: registration gate. A rejected solve short-circuits to an INDETERMINATE report.
  {
    StageClock clock(result.timings, PipelineStage::registration_gate);
    auto gate = registration::evaluate_gate(*metrics, config.registration_gate());
    if (!gate) {
      return Result<InspectionResult>::failure(std::move(gate).error());
    }
    if (!gate.value().passed) {
      result.verdict = Verdict::indeterminate;
      result.diagnostics.push_back(Diagnostic{ErrorCode::registration_failed,
                                              PipelineStage::registration_gate,
                                              gate.value().message,
                                              {}});
      return Result<InspectionResult>::success(std::move(result));
    }
  }

  // P06: transform the scan into the reference frame.
  std::optional<OwnedSurface> aligned_scan;
  {
    StageClock clock(result.timings, PipelineStage::transform);
    auto transformed =
        normalize_surface(scan_prepared->surface(), reference_prepared->surface().frame(),
                          metrics->final_transform());
    if (!transformed) {
      return Result<InspectionResult>::failure(std::move(transformed).error());
    }
    aligned_scan.emplace(std::move(transformed).value());
  }

  // P07: compare both directions.
  auto deviation_result = comparison::compute_deviation_field(
      reference_prepared->surface(), reference_prepared->boundary(), aligned_scan->view(),
      config.comparison());
  if (!deviation_result) {
    return Result<InspectionResult>::failure(std::move(deviation_result).error());
  }
  comparison::DeviationField deviation_field = std::move(deviation_result).value();
  auto coverage_result =
      comparison::compute_coverage_field(reference_prepared->surface(), aligned_scan->view(),
                                         scan_prepared->boundary(), config.comparison());
  if (!coverage_result) {
    return Result<InspectionResult>::failure(std::move(coverage_result).error());
  }
  comparison::CoverageField coverage_field = std::move(coverage_result).value();

  const auto deviation_statistics = comparison::summarize_deviation(deviation_field);
  const auto coverage_summary = comparison::summarize_coverage(coverage_field);

  // Map internal statistics onto the public report.
  result.deviations.valid_count = deviation_statistics.valid_count;
  result.deviations.mean_signed_mm = deviation_statistics.mean_signed_mm;
  result.deviations.rms_mm = deviation_statistics.rms_mm;
  result.deviations.max_abs_mm = deviation_statistics.max_abs_mm;
  result.deviations.p95_abs_mm = deviation_statistics.p95_abs_mm;
  result.deviations.input_invalid = deviation_statistics.input_invalid;
  result.deviations.no_neighbor = deviation_statistics.no_neighbor;
  result.deviations.normal_missing = deviation_statistics.normal_missing;
  result.deviations.normal_mismatch = deviation_statistics.normal_mismatch;
  result.deviations.reference_boundary = deviation_statistics.reference_boundary;

  result.coverage.valid_count = coverage_summary.valid_count;
  result.coverage.covered_count = coverage_summary.covered_count;
  result.coverage.coverage_ratio = coverage_summary.coverage_ratio;
  result.coverage.input_invalid = coverage_summary.input_invalid;
  result.coverage.no_neighbor = coverage_summary.no_neighbor;
  result.coverage.scan_boundary = coverage_summary.scan_boundary;

  // P08: coverage gate. Insufficient coverage forces INDETERMINATE but keeps the statistics.
  bool coverage_failed = false;
  {
    StageClock clock(result.timings, PipelineStage::coverage_gate);
    if (coverage_summary.coverage_ratio < config.comparison().min_valid_coverage_ratio) {
      coverage_failed = true;
      result.diagnostics.push_back(Diagnostic{ErrorCode::insufficient_coverage,
                                              PipelineStage::coverage_gate,
                                              "coverage ratio is below the minimum",
                                              {}});
    }
  }

  // P09: detect defects and measure regions.
  bool has_reject = false;
  {
    StageClock clock(result.timings, PipelineStage::detect);
    auto deviation_clusters = detection::cluster_deviation_defects(
        aligned_scan->view(), deviation_field, config.detection());
    if (!deviation_clusters) {
      return Result<InspectionResult>::failure(std::move(deviation_clusters).error());
    }
    auto missing_clusters = detection::cluster_missing_material(reference_prepared->surface(),
                                                                coverage_field, config.detection());
    if (!missing_clusters) {
      return Result<InspectionResult>::failure(std::move(missing_clusters).error());
    }

    std::uint32_t next_id = 0U;
    for (const auto& cluster : deviation_clusters.value()) {
      auto region = detection::measure_deviation_region(cluster, aligned_scan->view(),
                                                        deviation_field, next_id);
      if (!region) {
        return Result<InspectionResult>::failure(std::move(region).error());
      }
      DefectRegion report;
      report.id = region.value().id;
      report.type = to_public_type(region.value().type);
      report.point_count = region.value().point_count;
      report.centroid = region.value().centroid;
      report.aabb_min = region.value().aabb_min;
      report.aabb_max = region.value().aabb_max;
      report.max_abs_mm = region.value().max_abs_mm;
      report.mean_mm = region.value().mean_mm;
      report.rms_mm = region.value().rms_mm;
      report.p95_abs_mm = region.value().p95_abs_mm;
      report.estimated_area_mm2 = region.value().estimated_area_mm2;
      report.area_is_approximate = region.value().area_is_approximate;
      report.severity = default_severity(report.type, report.max_abs_mm, config.detection());
      has_reject = has_reject || report.severity == Severity::reject;
      result.regions.push_back(std::move(report));
      ++next_id;
    }
    for (const auto& cluster : missing_clusters.value()) {
      auto region =
          detection::measure_missing_region(cluster, reference_prepared->surface(), next_id);
      if (!region) {
        return Result<InspectionResult>::failure(std::move(region).error());
      }
      DefectRegion report;
      report.id = region.value().id;
      report.type = to_public_type(region.value().type);
      report.point_count = region.value().point_count;
      report.centroid = region.value().centroid;
      report.aabb_min = region.value().aabb_min;
      report.aabb_max = region.value().aabb_max;
      report.max_abs_mm = region.value().max_abs_mm;
      report.mean_mm = region.value().mean_mm;
      report.rms_mm = region.value().rms_mm;
      report.p95_abs_mm = region.value().p95_abs_mm;
      report.estimated_area_mm2 = region.value().estimated_area_mm2;
      report.area_is_approximate = region.value().area_is_approximate;
      report.severity = default_severity(report.type, report.max_abs_mm, config.detection());
      has_reject = has_reject || report.severity == Severity::reject;
      result.regions.push_back(std::move(report));
      ++next_id;
    }
  }

  // P10: finalize the verdict. No trustworthy deviation evidence (for example when normals cannot
  // be oriented) must never produce a PASS; only a complete comparison can pass.
  if (coverage_failed || result.deviations.valid_count == 0U) {
    result.verdict = Verdict::indeterminate;
  } else {
    result.verdict = has_reject ? Verdict::fail : Verdict::pass;
  }

  return Result<InspectionResult>::success(std::move(result));
}

} // namespace

struct InspectionPipeline::Impl {
  explicit Impl(ValidatedInspectionConfig config) : config(std::move(config)) {}
  ValidatedInspectionConfig config;
};

InspectionPipeline::~InspectionPipeline() = default;

InspectionPipeline::InspectionPipeline(ValidatedInspectionConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

Result<InspectionPipeline> InspectionPipeline::create(InspectionConfig config) noexcept {
  try {
    auto validated = validate_config(std::move(config));
    if (!validated) {
      return Result<InspectionPipeline>::failure(std::move(validated).error());
    }
    return Result<InspectionPipeline>::success(InspectionPipeline(std::move(validated).value()));
  } catch (const std::exception& exception) {
    return Result<InspectionPipeline>::failure(
        pipeline_error(ErrorCode::internal_error, exception.what()));
  } catch (...) {
    return Result<InspectionPipeline>::failure(
        pipeline_error(ErrorCode::internal_error, "unknown exception"));
  }
}

Result<InspectionResult> InspectionPipeline::run(SurfaceView reference, SurfaceView scan,
                                                 const InspectionRequest& request) const noexcept {
  try {
    return run_impl(impl_->config, reference, scan, request);
  } catch (const std::exception& exception) {
    return Result<InspectionResult>::failure(
        pipeline_error(ErrorCode::internal_error, exception.what()));
  } catch (...) {
    return Result<InspectionResult>::failure(
        pipeline_error(ErrorCode::internal_error, "unknown exception"));
  }
}

} // namespace pointcloud_ad
