#include "comparison_statistics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace pointcloud_ad::comparison {

DeviationStatistics summarize_deviation(const DeviationField& field) noexcept {
  DeviationStatistics statistics;
  std::vector<double> absolute_deviations;
  absolute_deviations.reserve(field.valid_count());

  double signed_sum = 0.0;
  double squared_sum = 0.0;
  double maximum_absolute = 0.0;

  for (const DeviationSample& sample : field.samples()) {
    switch (sample.reason) {
    case DeviationReason::valid:
      ++statistics.valid_count;
      signed_sum += sample.signed_mm;
      squared_sum += sample.signed_mm * sample.signed_mm;
      maximum_absolute = std::max(maximum_absolute, std::abs(sample.signed_mm));
      absolute_deviations.push_back(std::abs(sample.signed_mm));
      break;
    case DeviationReason::input_invalid:
      ++statistics.input_invalid;
      break;
    case DeviationReason::no_neighbor:
      ++statistics.no_neighbor;
      break;
    case DeviationReason::normal_missing:
      ++statistics.normal_missing;
      break;
    case DeviationReason::normal_mismatch:
      ++statistics.normal_mismatch;
      break;
    case DeviationReason::reference_boundary:
      ++statistics.reference_boundary;
      break;
    }
  }

  if (statistics.valid_count > 0U) {
    statistics.mean_signed_mm = signed_sum / static_cast<double>(statistics.valid_count);
    statistics.rms_mm = std::sqrt(squared_sum / static_cast<double>(statistics.valid_count));
    statistics.max_abs_mm = maximum_absolute;

    std::sort(absolute_deviations.begin(), absolute_deviations.end());
    const std::size_t percentile_index =
        static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(absolute_deviations.size())));
    const std::size_t clamped =
        percentile_index == 0U ? 0U
                               : std::min(percentile_index - 1U, absolute_deviations.size() - 1U);
    statistics.p95_abs_mm = absolute_deviations[clamped];
  }

  return statistics;
}

CoverageSummary summarize_coverage(const CoverageField& field) noexcept {
  CoverageSummary summary;
  summary.covered_count = field.covered_count();
  summary.valid_count = field.valid_count();
  summary.coverage_ratio = field.coverage_ratio();

  for (const CoverageSample& sample : field.samples()) {
    switch (sample.reason) {
    case CoverageReason::covered:
      break;
    case CoverageReason::input_invalid:
      ++summary.input_invalid;
      break;
    case CoverageReason::no_neighbor:
      ++summary.no_neighbor;
      break;
    case CoverageReason::scan_boundary:
      ++summary.scan_boundary;
      break;
    }
  }

  return summary;
}

} // namespace pointcloud_ad::comparison
