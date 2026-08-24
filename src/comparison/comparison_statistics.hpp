#pragma once

#include "coverage_field.hpp"
#include "deviation_field.hpp"

#include <cstddef>
#include <cstdint>

namespace pointcloud_ad::comparison {

// Aggregated statistics over a deviation field. All length statistics cover valid samples only;
// invalid samples are reported through the reason counters and never folded into the deviation
// statistics.
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

// Aggregated coverage summary over a coverage field.
struct CoverageSummary final {
  std::size_t valid_count{};
  std::size_t covered_count{};
  double coverage_ratio{};
  std::size_t input_invalid{};
  std::size_t no_neighbor{};
  std::size_t scan_boundary{};
};

// Computes deterministic deviation statistics, including the 95th-percentile absolute deviation.
[[nodiscard]] DeviationStatistics summarize_deviation(const DeviationField& field) noexcept;

// Computes deterministic coverage summary counters and ratio.
[[nodiscard]] CoverageSummary summarize_coverage(const CoverageField& field) noexcept;

} // namespace pointcloud_ad::comparison
