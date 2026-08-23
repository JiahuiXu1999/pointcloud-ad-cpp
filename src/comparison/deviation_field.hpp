#pragma once

#include <cstddef>
#include <cstdint>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/surface.hpp>
#include <span>
#include <vector>

namespace pointcloud_ad::comparison {

// Why a scan point's deviation sample is unusable for defect decisions. `valid` is the only state
// whose signed deviation is trustworthy; every other state must not be interpreted as zero
// deviation downstream.
enum class DeviationReason : std::uint8_t {
  valid,              // Signed deviation is trustworthy.
  input_invalid,      // Scan point itself is masked out.
  no_neighbor,        // No reference point within the search radius.
  normal_missing,     // Reference neighbour has no usable normal.
  normal_mismatch,    // Scan/reference normals diverge beyond the configured angle.
  reference_boundary, // Reference neighbour lies on a boundary.
};

struct DeviationSample final {
  double euclidean_mm{};
  double signed_mm{};
  double normal_angle_deg{};
  DeviationReason reason{DeviationReason::input_invalid};
};

// Per-storage-index deviation samples for an aligned scan, plus the count of trustworthy samples.
// Samples are parallel to the aligned scan's storage layout so downstream detection can address
// points by the same index.
class DeviationField final {
public:
  DeviationField(const DeviationField&) = delete;
  DeviationField& operator=(const DeviationField&) = delete;
  DeviationField(DeviationField&&) noexcept = default;
  DeviationField& operator=(DeviationField&&) noexcept = default;
  ~DeviationField() = default;

  DeviationField(std::vector<DeviationSample> samples, std::size_t valid_count) noexcept
      : samples_(std::move(samples)), valid_count_(valid_count) {}

  [[nodiscard]] std::span<const DeviationSample> samples() const noexcept {
    return samples_;
  }
  [[nodiscard]] std::size_t size() const noexcept {
    return samples_.size();
  }
  [[nodiscard]] std::size_t valid_count() const noexcept {
    return valid_count_;
  }

private:
  std::vector<DeviationSample> samples_;
  std::size_t valid_count_{};
};

// Computes the scan-to-reference deviation field for an already-aligned scan. `reference_boundary`
// may be empty to skip boundary rejection, otherwise it must span the reference's storage layout
// with non-zero entries marking boundary points. Both surfaces must be normalized millimetres and
// the reference must carry normals for every valid point.
[[nodiscard]] Result<DeviationField>
compute_deviation_field(SurfaceView reference, std::span<const std::uint8_t> reference_boundary,
                        SurfaceView aligned_scan, const ValidatedComparisonConfig& config) noexcept;

} // namespace pointcloud_ad::comparison
