#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/status.hpp>
#include <string>
#include <vector>

namespace pointcloud_ad {

// Business conclusion of a completed inspection. It is deliberately independent of process exit
// codes; a program-level failure is reported through Result<T> instead.
enum class Verdict : std::uint8_t { pass, fail, indeterminate };

enum class DefectType : std::uint8_t { dent, bump, missing_material };

enum class Severity : std::uint8_t { info, warning, reject };

// Quantitative registration outcome. The final pose is scan-to-reference; the overlap ratio and
// inlier residual describe the accepted correspondences. The final pose is absent when registration
// short-circuited before producing a transform.
struct RegistrationResult final {
  std::optional<RigidTransform> initial_pose;
  std::optional<RigidTransform> final_pose;
  std::uint32_t iterations{};
  bool converged{false};
  std::uint64_t valid_pairs{};
  double overlap_ratio{};
  double inlier_rmse_mm{};
  std::string termination_reason;
};

// Aggregate coverage of the reference surface by the aligned scan.
struct CoverageSummary final {
  std::size_t valid_count{};
  std::size_t covered_count{};
  double coverage_ratio{};
  std::size_t input_invalid{};
  std::size_t no_neighbor{};
  std::size_t scan_boundary{};
};

// Aggregate signed-deviation statistics over the aligned scan. Length statistics cover valid
// samples only.
struct DeviationStatistics final {
  std::size_t valid_count{};
  double mean_signed_mm{};
  double rms_mm{};
  double max_abs_mm{};
  double p95_abs_mm{};
  std::size_t input_invalid{};
  std::size_t no_neighbor{};
  std::size_t normal_missing{};
  std::size_t normal_mismatch{};
  std::size_t reference_boundary{};
};

// Measured summary of one defect region. The area is a point-cloud estimate and is always flagged
// approximate.
struct DefectRegion final {
  std::uint32_t id{};
  DefectType type{DefectType::dent};
  std::size_t point_count{};
  Vec3d centroid{};
  Vec3d aabb_min{};
  Vec3d aabb_max{};
  double max_abs_mm{};
  double mean_mm{};
  double rms_mm{};
  double p95_abs_mm{};
  std::optional<double> estimated_area_mm2{};
  bool area_is_approximate{true};
  Severity severity{Severity::info};
};

// Elapsed time of one pipeline stage in microseconds.
struct StageTiming final {
  PipelineStage stage{PipelineStage::none};
  std::uint64_t duration_us{};
};

// Reproducibility and origin metadata for one inspection run.
struct Provenance final {
  std::string schema_version;
  std::string run_id;
  std::string profile;
  std::string generator_version;
  std::string timestamp_utc;
  bool deterministic{true};
  std::uint32_t thread_count{1};
  std::uint64_t random_seed{};
};

// Non-fatal diagnostic attached to a completed inspection report.
struct Diagnostic final {
  ErrorCode code{ErrorCode::internal_error};
  PipelineStage stage{PipelineStage::none};
  std::string message;
  std::map<std::string, std::string> context;
};

// The full, versioned result of one inspection. `verdict` describes the business conclusion of a
// completed report; a `Result<InspectionResult>` error means the call itself did not complete.
struct InspectionResult final {
  Verdict verdict{Verdict::indeterminate};
  RegistrationResult registration;
  CoverageSummary coverage;
  DeviationStatistics deviations;
  std::vector<DefectRegion> regions;
  std::vector<StageTiming> timings;
  Provenance provenance;
  std::vector<Diagnostic> diagnostics;
};

} // namespace pointcloud_ad
