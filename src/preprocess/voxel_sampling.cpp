#include "voxel_sampling.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

namespace pointcloud_ad::preprocess {
namespace {

struct VoxelKey final {
  std::int64_t x{};
  std::int64_t y{};
  std::int64_t z{};

  friend bool operator<(const VoxelKey& left, const VoxelKey& right) noexcept {
    return std::tie(left.x, left.y, left.z) < std::tie(right.x, right.y, right.z);
  }
};

struct Accumulator final {
  Vec3d point_sum{};
  Vec3d normal_sum{};
  std::vector<std::size_t> source_indices;
};

[[nodiscard]] Error sampling_error(ErrorCode code, std::string field, std::string reason) {
  return Error{code,
               PipelineStage::preprocess,
               "voxel sampling failed",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

[[nodiscard]] Result<std::int64_t> coordinate_key(float coordinate, double voxel_size_mm) {
  const double value = std::floor(static_cast<double>(coordinate) / voxel_size_mm);
  constexpr double limit = 0x1p63;
  if (!std::isfinite(value) || value < -limit || value >= limit) {
    return Result<std::int64_t>::failure(sampling_error(
        ErrorCode::invalid_input, "surface.points", "coordinate produces an invalid voxel key"));
  }
  return Result<std::int64_t>::success(static_cast<std::int64_t>(value));
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

[[nodiscard]] Result<VoxelSampleResult> voxel_sample_impl(SurfaceView surface,
                                                          double voxel_size_mm) {
  if (surface.unit() != LengthUnit::millimeter) {
    return Result<VoxelSampleResult>::failure(sampling_error(
        ErrorCode::invalid_input, "surface.unit", "must be normalized to millimetres"));
  }
  if (!std::isfinite(voxel_size_mm) || voxel_size_mm <= 0.0) {
    return Result<VoxelSampleResult>::failure(sampling_error(
        ErrorCode::invalid_argument, "voxel_size_mm", "must be finite and positive"));
  }

  std::map<VoxelKey, Accumulator> voxels;
  std::optional<Error> key_error;
  for_each_logical_index(surface, [&](std::size_t index) {
    if (key_error || (!surface.valid().empty() && surface.valid()[index] == 0U)) {
      return;
    }
    auto key_x = coordinate_key(surface.points()[index].x, voxel_size_mm);
    auto key_y = coordinate_key(surface.points()[index].y, voxel_size_mm);
    auto key_z = coordinate_key(surface.points()[index].z, voxel_size_mm);
    if (!key_x || !key_y || !key_z) {
      if (!key_x) {
        key_error = std::move(key_x).error();
      } else if (!key_y) {
        key_error = std::move(key_y).error();
      } else {
        key_error = std::move(key_z).error();
      }
      return;
    }
    auto& voxel = voxels[VoxelKey{key_x.value(), key_y.value(), key_z.value()}];
    const auto point = surface.points()[index];
    voxel.point_sum.x += point.x;
    voxel.point_sum.y += point.y;
    voxel.point_sum.z += point.z;
    if (!surface.normals().empty()) {
      const auto normal = surface.normals()[index];
      voxel.normal_sum.x += normal.x;
      voxel.normal_sum.y += normal.y;
      voxel.normal_sum.z += normal.z;
    }
    voxel.source_indices.push_back(index);
  });
  if (key_error) {
    return Result<VoxelSampleResult>::failure(std::move(*key_error));
  }
  if (voxels.empty()) {
    return Result<VoxelSampleResult>::failure(sampling_error(
        ErrorCode::invalid_input, "surface.valid", "must contain at least one valid sample"));
  }

  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  std::vector<std::size_t> source_offsets;
  std::vector<std::size_t> source_indices;
  points.reserve(voxels.size());
  if (!surface.normals().empty()) {
    normals.reserve(voxels.size());
  }
  source_offsets.reserve(voxels.size() + 1U);
  source_indices.reserve(surface.size());
  source_offsets.push_back(0U);
  for (auto& [key, voxel] : voxels) {
    static_cast<void>(key);
    const double divisor = static_cast<double>(voxel.source_indices.size());
    points.push_back(Vec3f{static_cast<float>(voxel.point_sum.x / divisor),
                           static_cast<float>(voxel.point_sum.y / divisor),
                           static_cast<float>(voxel.point_sum.z / divisor)});
    if (!surface.normals().empty()) {
      const double length = std::sqrt(voxel.normal_sum.x * voxel.normal_sum.x +
                                      voxel.normal_sum.y * voxel.normal_sum.y +
                                      voxel.normal_sum.z * voxel.normal_sum.z);
      if (length > 0.0 && std::isfinite(length)) {
        normals.push_back(Vec3f{static_cast<float>(voxel.normal_sum.x / length),
                                static_cast<float>(voxel.normal_sum.y / length),
                                static_cast<float>(voxel.normal_sum.z / length)});
      } else {
        normals.push_back(Vec3f{});
      }
    }
    source_indices.insert(source_indices.end(), voxel.source_indices.begin(),
                          voxel.source_indices.end());
    source_offsets.push_back(source_indices.size());
  }

  auto sampled = OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                                      LengthUnit::millimeter, surface.frame());
  if (!sampled) {
    return Result<VoxelSampleResult>::failure(std::move(sampled).error());
  }
  return Result<VoxelSampleResult>::success(VoxelSampleResult(
      std::move(sampled).value(), std::move(source_offsets), std::move(source_indices)));
}

} // namespace

Result<VoxelSampleResult> voxel_sample(SurfaceView surface, double voxel_size_mm) noexcept {
  try {
    return voxel_sample_impl(surface, voxel_size_mm);
  } catch (const std::exception& exception) {
    return Result<VoxelSampleResult>::failure(
        sampling_error(ErrorCode::internal_error, "exception", exception.what()));
  } catch (...) {
    return Result<VoxelSampleResult>::failure(
        sampling_error(ErrorCode::internal_error, "exception", "unknown exception"));
  }
}

} // namespace pointcloud_ad::preprocess
