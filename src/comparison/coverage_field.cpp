#include "coverage_field.hpp"

#include "pcl_comparison_backend.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_ad::comparison {
namespace {

[[nodiscard]] Error coverage_error(ErrorCode code, std::string field, std::string reason) {
  return Error{code,
               PipelineStage::compare,
               "coverage field computation failed",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

[[nodiscard]] bool valid_at(SurfaceView surface, std::size_t index) noexcept {
  return surface.valid().empty() || surface.valid()[index] == 1U;
}

template <typename Function> void for_each_logical_index(SurfaceView surface, Function&& function) {
  if (!surface.grid()) {
    for (std::size_t index = 0; index < surface.storage_size(); ++index) {
      function(index);
    }
    return;
  }
  const auto& grid = *surface.grid();
  for (std::size_t row = 0; row < grid.height; ++row) {
    for (std::size_t column = 0; column < grid.width; ++column) {
      function(row * static_cast<std::size_t>(grid.row_stride) + column);
    }
  }
}

[[nodiscard]] Result<CoverageField> compute_impl(SurfaceView reference, SurfaceView aligned_scan,
                                                 std::span<const std::uint8_t> scan_boundary,
                                                 const ValidatedComparisonConfig& config) {
  if (reference.unit() != LengthUnit::millimeter) {
    return Result<CoverageField>::failure(coverage_error(ErrorCode::invalid_input, "reference.unit",
                                                         "must be normalized to millimetres"));
  }
  if (aligned_scan.unit() != LengthUnit::millimeter) {
    return Result<CoverageField>::failure(coverage_error(
        ErrorCode::invalid_input, "aligned_scan.unit", "must be normalized to millimetres"));
  }
  if (!scan_boundary.empty() && scan_boundary.size() != aligned_scan.storage_size()) {
    return Result<CoverageField>::failure(coverage_error(
        ErrorCode::invalid_input, "scan_boundary", "must span the aligned scan storage layout"));
  }

  // Coverage queries reference points against a scan tree, so the search roles are swapped
  // relative to the deviation field.
  auto nearest = backends::pcl_backend::nearest_neighbors(aligned_scan, reference,
                                                          config.max_search_distance_mm);
  if (!nearest) {
    return Result<CoverageField>::failure(std::move(nearest).error());
  }
  const auto& neighbor_index = nearest.value().neighbor_index;
  const auto& distances = nearest.value().distance_mm;

  std::vector<CoverageSample> samples(reference.storage_size());
  std::size_t covered_count = 0U;
  std::size_t valid_count = 0U;
  for_each_logical_index(reference, [&](std::size_t index) {
    CoverageSample& sample = samples[index];
    if (!valid_at(reference, index)) {
      sample.reason = CoverageReason::input_invalid;
      return;
    }
    ++valid_count;
    const auto neighbor = neighbor_index[index];
    if (neighbor < 0) {
      sample.reason = CoverageReason::no_neighbor;
      return;
    }
    sample.distance_mm = static_cast<double>(distances[index]);
    if (!scan_boundary.empty() && scan_boundary[static_cast<std::size_t>(neighbor)] != 0U) {
      sample.reason = CoverageReason::scan_boundary;
      return;
    }
    sample.reason = CoverageReason::covered;
    ++covered_count;
  });

  return Result<CoverageField>::success(
      CoverageField(std::move(samples), covered_count, valid_count));
}

} // namespace

Result<CoverageField> compute_coverage_field(SurfaceView reference, SurfaceView aligned_scan,
                                             std::span<const std::uint8_t> scan_boundary,
                                             const ValidatedComparisonConfig& config) noexcept {
  try {
    return compute_impl(reference, aligned_scan, scan_boundary, config);
  } catch (const std::exception& exception) {
    return Result<CoverageField>::failure(
        coverage_error(ErrorCode::internal_error, "exception", exception.what()));
  } catch (...) {
    return Result<CoverageField>::failure(
        coverage_error(ErrorCode::internal_error, "exception", "unknown exception"));
  }
}

} // namespace pointcloud_ad::comparison
