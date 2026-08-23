#pragma once

#include <cstddef>
#include <pointcloud_ad/surface.hpp>
#include <span>
#include <utility>
#include <vector>

namespace pointcloud_ad::preprocess {

class VoxelSampleResult final {
public:
  VoxelSampleResult(const VoxelSampleResult&) = delete;
  VoxelSampleResult& operator=(const VoxelSampleResult&) = delete;
  VoxelSampleResult(VoxelSampleResult&&) noexcept = default;
  VoxelSampleResult& operator=(VoxelSampleResult&&) noexcept = default;
  ~VoxelSampleResult() = default;

  VoxelSampleResult(OwnedSurface surface, std::vector<std::size_t> source_offsets,
                    std::vector<std::size_t> source_indices) noexcept
      : surface_(std::move(surface)), source_offsets_(std::move(source_offsets)),
        source_indices_(std::move(source_indices)) {}

  [[nodiscard]] SurfaceView surface() const noexcept {
    return surface_.view();
  }
  [[nodiscard]] std::span<const std::size_t> source_offsets() const noexcept {
    return source_offsets_;
  }
  [[nodiscard]] std::span<const std::size_t> source_indices() const noexcept {
    return source_indices_;
  }
  [[nodiscard]] std::span<const std::size_t> sources_for(std::size_t sampled_index) const noexcept {
    const auto begin = source_offsets_[sampled_index];
    const auto end = source_offsets_[sampled_index + 1U];
    return std::span<const std::size_t>(source_indices_).subspan(begin, end - begin);
  }

private:
  OwnedSurface surface_;
  std::vector<std::size_t> source_offsets_;
  std::vector<std::size_t> source_indices_;
};

[[nodiscard]] Result<VoxelSampleResult> voxel_sample(SurfaceView surface,
                                                     double voxel_size_mm) noexcept;

} // namespace pointcloud_ad::preprocess
