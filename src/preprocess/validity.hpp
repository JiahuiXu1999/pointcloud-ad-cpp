#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <pointcloud_ad/surface.hpp>
#include <span>
#include <utility>
#include <vector>

namespace pointcloud_ad::preprocess {

struct AxisAlignedRoi final {
  Vec3f minimum_mm;
  Vec3f maximum_mm;
};

enum class InvalidReason : std::uint8_t {
  none,
  input_masked,
  non_finite_point,
  non_finite_normal,
  outside_roi,
  row_padding
};

class ValidityMaskResult final {
public:
  ValidityMaskResult(const ValidityMaskResult&) = delete;
  ValidityMaskResult& operator=(const ValidityMaskResult&) = delete;
  ValidityMaskResult(ValidityMaskResult&&) noexcept = default;
  ValidityMaskResult& operator=(ValidityMaskResult&&) noexcept = default;
  ~ValidityMaskResult() = default;

  [[nodiscard]] SurfaceView surface() const noexcept {
    return surface_.view();
  }
  [[nodiscard]] std::span<const InvalidReason> reasons() const noexcept {
    return reasons_;
  }
  [[nodiscard]] std::size_t valid_count() const noexcept {
    return valid_count_;
  }

  ValidityMaskResult(OwnedSurface surface, std::vector<InvalidReason> reasons,
                     std::size_t valid_count) noexcept
      : surface_(std::move(surface)), reasons_(std::move(reasons)), valid_count_(valid_count) {}

private:
  OwnedSurface surface_;
  std::vector<InvalidReason> reasons_;
  std::size_t valid_count_{};
};

[[nodiscard]] Result<ValidityMaskResult>
apply_validity_mask(SurfaceView surface, std::optional<AxisAlignedRoi> roi = std::nullopt) noexcept;

} // namespace pointcloud_ad::preprocess
