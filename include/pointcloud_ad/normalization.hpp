#pragma once

#include <cmath>
#include <exception>
#include <limits>
#include <optional>
#include <pointcloud_ad/surface.hpp>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_ad {

namespace detail {

[[nodiscard]] inline Error normalization_error(std::string field, std::string reason) {
  return Error{ErrorCode::invalid_input,
               PipelineStage::normalize,
               "surface normalization failed",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

[[nodiscard]] inline Vec3d rotate_vector(const RigidTransform& transform, Vec3f vector) noexcept {
  const auto& matrix = transform.matrix();
  return Vec3d{matrix[0] * vector.x + matrix[1] * vector.y + matrix[2] * vector.z,
               matrix[4] * vector.x + matrix[5] * vector.y + matrix[6] * vector.z,
               matrix[8] * vector.x + matrix[9] * vector.y + matrix[10] * vector.z};
}

[[nodiscard]] inline bool fits_float(Vec3d vector) noexcept {
  constexpr double maximum = static_cast<double>(std::numeric_limits<float>::max());
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z) &&
         std::abs(vector.x) <= maximum && std::abs(vector.y) <= maximum &&
         std::abs(vector.z) <= maximum;
}

[[nodiscard]] inline Result<OwnedSurface>
normalize_surface_impl(SurfaceView surface, FrameId target_frame,
                       const std::optional<RigidTransform>& transform) {
  auto scale_result = millimeters_per_unit(surface.unit());
  if (!scale_result) {
    return Result<OwnedSurface>::failure(std::move(scale_result).error());
  }
  const double scale = scale_result.value();

  const bool frame_changes = surface.frame() != target_frame;
  if (frame_changes && !transform) {
    return Result<OwnedSurface>::failure(
        normalization_error("transform", "is required when source and target frames differ"));
  }
  if (transform &&
      (transform->source_frame() != surface.frame() || transform->target_frame() != target_frame)) {
    return Result<OwnedSurface>::failure(
        normalization_error("transform", "frame direction must match surface -> target"));
  }

  std::vector<Vec3f> points(surface.points().begin(), surface.points().end());
  std::vector<Vec3f> normals(surface.normals().begin(), surface.normals().end());
  std::vector<std::uint8_t> valid(surface.valid().begin(), surface.valid().end());

  const auto normalize_index = [&](std::size_t index) -> Result<std::size_t> {
    const bool sample_is_valid = valid.empty() || valid[index] == 1U;
    const Vec3f source = points[index];
    Vec3d normalized{static_cast<double>(source.x) * scale, static_cast<double>(source.y) * scale,
                     static_cast<double>(source.z) * scale};
    if (transform) {
      normalized = transform->apply(normalized);
    }
    if (sample_is_valid && !fits_float(normalized)) {
      return Result<std::size_t>::failure(
          normalization_error("points", "normalized valid point is not representable as Vec3f"));
    }
    points[index] = Vec3f{static_cast<float>(normalized.x), static_cast<float>(normalized.y),
                          static_cast<float>(normalized.z)};

    if (!normals.empty()) {
      Vec3d normal{normals[index].x, normals[index].y, normals[index].z};
      if (transform) {
        normal = rotate_vector(*transform, normals[index]);
      }
      if (sample_is_valid && !fits_float(normal)) {
        return Result<std::size_t>::failure(
            normalization_error("normals", "normalized valid normal is not representable"));
      }
      normals[index] = Vec3f{static_cast<float>(normal.x), static_cast<float>(normal.y),
                             static_cast<float>(normal.z)};
    }
    return Result<std::size_t>::success(index);
  };

  if (!surface.grid()) {
    for (std::size_t index = 0; index < points.size(); ++index) {
      auto result = normalize_index(index);
      if (!result) {
        return Result<OwnedSurface>::failure(std::move(result).error());
      }
    }
  } else {
    for (std::size_t row = 0; row < surface.grid()->height; ++row) {
      for (std::size_t column = 0; column < surface.grid()->width; ++column) {
        const std::size_t index =
            row * static_cast<std::size_t>(surface.grid()->row_stride) + column;
        auto result = normalize_index(index);
        if (!result) {
          return Result<OwnedSurface>::failure(std::move(result).error());
        }
      }
    }
  }

  return OwnedSurface::create(std::move(points), std::move(normals), std::move(valid),
                              surface.grid(), LengthUnit::millimeter, std::move(target_frame));
}

} // namespace detail

// Converts coordinates exactly once to millimetres and, when requested, applies a matching
// source-to-target rigid transform. Normals are rotated but never translated or unit-scaled.
[[nodiscard]] inline Result<OwnedSurface>
normalize_surface(SurfaceView surface, FrameId target_frame,
                  std::optional<RigidTransform> transform = std::nullopt) noexcept {
  try {
    return detail::normalize_surface_impl(surface, std::move(target_frame), transform);
  } catch (const std::exception& exception) {
    return Result<OwnedSurface>::failure(Error{ErrorCode::internal_error,
                                               PipelineStage::normalize,
                                               "unexpected exception while normalizing surface",
                                               {{"exception", exception.what()}}});
  } catch (...) {
    return Result<OwnedSurface>::failure(Error{ErrorCode::internal_error,
                                               PipelineStage::normalize,
                                               "unknown exception while normalizing surface",
                                               {}});
  }
}

} // namespace pointcloud_ad
