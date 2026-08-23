#pragma once

#include <cstddef>
#include <cstdint>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/surface.hpp>
#include <span>
#include <vector>

namespace pointcloud_ad::comparison {

// Why a reference point is not considered covered by the aligned scan. Only `covered` contributes
// to the coverage ratio; every other state keeps the point out of the covered set.
enum class CoverageReason : std::uint8_t {
  covered,       // A scan neighbour was found and is usable.
  input_invalid, // Reference point itself is masked out.
  no_neighbor,   // No scan point within the search radius (missing-material candidate).
  scan_boundary, // The nearest scan point lies on a boundary, so coverage is unreliable.
};

struct CoverageSample final {
  double distance_mm{};
  CoverageReason reason{CoverageReason::input_invalid};
};

// Per-storage-index coverage samples for a reference surface, plus the coverage ratio used by the
// downstream coverage gate. Samples are parallel to the reference storage layout.
class CoverageField final {
public:
  CoverageField(const CoverageField&) = delete;
  CoverageField& operator=(const CoverageField&) = delete;
  CoverageField(CoverageField&&) noexcept = default;
  CoverageField& operator=(CoverageField&&) noexcept = default;
  ~CoverageField() = default;

  CoverageField(std::vector<CoverageSample> samples, std::size_t covered_count,
                std::size_t valid_count) noexcept
      : samples_(std::move(samples)), covered_count_(covered_count), valid_count_(valid_count) {}

  [[nodiscard]] std::span<const CoverageSample> samples() const noexcept {
    return samples_;
  }
  [[nodiscard]] std::size_t size() const noexcept {
    return samples_.size();
  }
  [[nodiscard]] std::size_t covered_count() const noexcept {
    return covered_count_;
  }
  [[nodiscard]] std::size_t valid_count() const noexcept {
    return valid_count_;
  }
  // Fraction of valid reference points that are covered, in [0, 1].
  [[nodiscard]] double coverage_ratio() const noexcept {
    return valid_count_ == 0U
               ? 0.0
               : static_cast<double>(covered_count_) / static_cast<double>(valid_count_);
  }

private:
  std::vector<CoverageSample> samples_;
  std::size_t covered_count_{};
  std::size_t valid_count_{};
};

// Computes the reference-to-scan coverage field. `scan_boundary` may be empty to skip boundary
// rejection, otherwise it must span the aligned scan's storage layout. Both surfaces must be
// normalized millimetres.
[[nodiscard]] Result<CoverageField>
compute_coverage_field(SurfaceView reference, SurfaceView aligned_scan,
                       std::span<const std::uint8_t> scan_boundary,
                       const ValidatedComparisonConfig& config) noexcept;

} // namespace pointcloud_ad::comparison
