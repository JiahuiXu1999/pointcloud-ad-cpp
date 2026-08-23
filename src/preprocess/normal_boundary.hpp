#pragma once

#include <cstddef>
#include <cstdint>
#include <pointcloud_ad/surface.hpp>
#include <span>
#include <utility>
#include <vector>

namespace pointcloud_ad::preprocess {

enum class NormalOrientationMode : std::uint8_t { unproven, align_direction, toward_viewpoint };

struct NormalOrientationHint final {
  NormalOrientationMode mode{NormalOrientationMode::unproven};
  Vec3f value{}; // Direction (unitless) or viewpoint position (mm), in the surface frame.
};

enum class NormalReason : std::uint8_t { none, input_invalid, normal_missing };

class NormalBoundaryResult final {
public:
  NormalBoundaryResult(const NormalBoundaryResult&) = delete;
  NormalBoundaryResult& operator=(const NormalBoundaryResult&) = delete;
  NormalBoundaryResult(NormalBoundaryResult&&) noexcept = default;
  NormalBoundaryResult& operator=(NormalBoundaryResult&&) noexcept = default;
  ~NormalBoundaryResult() = default;

  NormalBoundaryResult(OwnedSurface surface, std::vector<NormalReason> reasons,
                       std::vector<std::uint8_t> orientation_valid,
                       std::vector<std::uint8_t> boundary, bool orientation_proven) noexcept
      : surface_(std::move(surface)), reasons_(std::move(reasons)),
        orientation_valid_(std::move(orientation_valid)), boundary_(std::move(boundary)),
        orientation_proven_(orientation_proven) {}

  [[nodiscard]] SurfaceView surface() const noexcept {
    return surface_.view();
  }
  [[nodiscard]] std::span<const NormalReason> reasons() const noexcept {
    return reasons_;
  }
  [[nodiscard]] std::span<const std::uint8_t> orientation_valid() const noexcept {
    return orientation_valid_;
  }
  [[nodiscard]] std::span<const std::uint8_t> boundary() const noexcept {
    return boundary_;
  }
  [[nodiscard]] bool orientation_proven() const noexcept {
    return orientation_proven_;
  }

private:
  OwnedSurface surface_;
  std::vector<NormalReason> reasons_;
  std::vector<std::uint8_t> orientation_valid_;
  std::vector<std::uint8_t> boundary_;
  bool orientation_proven_{};
};

[[nodiscard]] Result<NormalBoundaryResult>
prepare_normals_and_boundaries(SurfaceView surface, double normal_radius_mm,
                               std::uint32_t normal_min_neighbors, double boundary_radius_mm,
                               NormalOrientationHint orientation = {}) noexcept;

} // namespace pointcloud_ad::preprocess
