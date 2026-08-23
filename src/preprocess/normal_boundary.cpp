#include "normal_boundary.hpp"

#include "pcl_preprocess_backend.hpp"

#include <cmath>
#include <exception>
#include <optional>
#include <string>
#include <utility>

namespace pointcloud_ad::preprocess {
namespace {

[[nodiscard]] Error normal_error(ErrorCode code, std::string field, std::string reason) {
  return Error{code,
               PipelineStage::preprocess,
               "normal and boundary preprocessing failed",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

[[nodiscard]] bool finite(Vec3f value) noexcept {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] double length(Vec3f value) noexcept {
  return std::sqrt(static_cast<double>(value.x) * value.x + static_cast<double>(value.y) * value.y +
                   static_cast<double>(value.z) * value.z);
}

[[nodiscard]] Vec3f normalized(Vec3f value) noexcept {
  const double magnitude = length(value);
  if (!(magnitude > 0.0) || !std::isfinite(magnitude)) {
    return Vec3f{};
  }
  return Vec3f{static_cast<float>(value.x / magnitude), static_cast<float>(value.y / magnitude),
               static_cast<float>(value.z / magnitude)};
}

[[nodiscard]] double dot(Vec3f left, Vec3f right) noexcept {
  return static_cast<double>(left.x) * right.x + static_cast<double>(left.y) * right.y +
         static_cast<double>(left.z) * right.z;
}

[[nodiscard]] Vec3f negate(Vec3f value) noexcept {
  return Vec3f{-value.x, -value.y, -value.z};
}

[[nodiscard]] bool input_valid(SurfaceView surface, std::size_t index) noexcept {
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

[[nodiscard]] std::vector<std::uint8_t> detect_grid_boundaries(SurfaceView surface,
                                                               double radius_mm) {
  std::vector<std::uint8_t> boundary(surface.storage_size(), 0U);
  const auto& grid = *surface.grid();
  constexpr int offsets[4][2]{{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (std::uint32_t row = 0; row < grid.height; ++row) {
    for (std::uint32_t column = 0; column < grid.width; ++column) {
      const auto index = static_cast<std::size_t>(row) * grid.row_stride + column;
      if (!input_valid(surface, index)) {
        continue;
      }
      for (const auto& offset : offsets) {
        const auto neighbor_row = static_cast<std::int64_t>(row) + offset[0];
        const auto neighbor_column = static_cast<std::int64_t>(column) + offset[1];
        if (neighbor_row < 0 || neighbor_column < 0 ||
            neighbor_row >= static_cast<std::int64_t>(grid.height) ||
            neighbor_column >= static_cast<std::int64_t>(grid.width)) {
          boundary[index] = 1U;
          break;
        }
        const auto neighbor_index = static_cast<std::size_t>(neighbor_row) * grid.row_stride +
                                    static_cast<std::size_t>(neighbor_column);
        if (!input_valid(surface, neighbor_index)) {
          boundary[index] = 1U;
          break;
        }
        const auto point = surface.points()[index];
        const auto neighbor = surface.points()[neighbor_index];
        const Vec3f delta{point.x - neighbor.x, point.y - neighbor.y, point.z - neighbor.z};
        if (length(delta) > radius_mm) {
          boundary[index] = 1U;
          break;
        }
      }
    }
  }
  return boundary;
}

[[nodiscard]] Result<NormalBoundaryResult>
prepare_impl(SurfaceView surface, double normal_radius_mm, std::uint32_t normal_min_neighbors,
             double boundary_radius_mm, NormalOrientationHint orientation) {
  if (surface.unit() != LengthUnit::millimeter) {
    return Result<NormalBoundaryResult>::failure(normal_error(
        ErrorCode::invalid_input, "surface.unit", "must be normalized to millimetres"));
  }
  if (!std::isfinite(normal_radius_mm) || normal_radius_mm <= 0.0) {
    return Result<NormalBoundaryResult>::failure(normal_error(
        ErrorCode::invalid_argument, "normal_radius_mm", "must be finite and positive"));
  }
  if (normal_min_neighbors < 3U) {
    return Result<NormalBoundaryResult>::failure(
        normal_error(ErrorCode::invalid_argument, "normal_min_neighbors", "must be at least 3"));
  }
  if (!std::isfinite(boundary_radius_mm) || boundary_radius_mm <= 0.0) {
    return Result<NormalBoundaryResult>::failure(normal_error(
        ErrorCode::invalid_argument, "boundary_radius_mm", "must be finite and positive"));
  }
  if (orientation.mode != NormalOrientationMode::unproven &&
      (!finite(orientation.value) || length(orientation.value) == 0.0)) {
    return Result<NormalBoundaryResult>::failure(normal_error(
        ErrorCode::invalid_argument, "orientation.value", "must be finite and non-zero"));
  }

  std::vector<Vec3f> normals(surface.storage_size());
  std::vector<std::uint8_t> normal_valid(surface.storage_size(), 0U);
  if (surface.normals().empty()) {
    auto estimated =
        backends::pcl_backend::estimate_normals(surface, normal_radius_mm, normal_min_neighbors);
    if (!estimated) {
      return Result<NormalBoundaryResult>::failure(std::move(estimated).error());
    }
    normals = std::move(estimated.value().normals);
    normal_valid = std::move(estimated.value().valid);
  } else {
    for_each_logical_index(surface, [&](std::size_t index) {
      if (!input_valid(surface, index)) {
        return;
      }
      normals[index] = normalized(surface.normals()[index]);
      normal_valid[index] = length(normals[index]) > 0.0 ? 1U : 0U;
    });
  }

  std::vector<std::uint8_t> output_mask(surface.storage_size(), 0U);
  std::vector<NormalReason> reasons(surface.storage_size(), NormalReason::input_invalid);
  std::vector<std::uint8_t> orientation_valid(surface.storage_size(), 0U);
  std::size_t valid_normal_count = 0U;
  std::size_t oriented_count = 0U;
  const auto direction = orientation.mode == NormalOrientationMode::align_direction
                             ? normalized(orientation.value)
                             : Vec3f{};
  for_each_logical_index(surface, [&](std::size_t index) {
    if (!input_valid(surface, index)) {
      return;
    }
    if (normal_valid[index] == 0U) {
      reasons[index] = NormalReason::normal_missing;
      return;
    }
    reasons[index] = NormalReason::none;
    output_mask[index] = 1U;
    ++valid_normal_count;
    if (orientation.mode == NormalOrientationMode::align_direction) {
      if (dot(normals[index], direction) < 0.0) {
        normals[index] = negate(normals[index]);
      }
      orientation_valid[index] = 1U;
    } else if (orientation.mode == NormalOrientationMode::toward_viewpoint) {
      const auto point = surface.points()[index];
      const Vec3f toward{orientation.value.x - point.x, orientation.value.y - point.y,
                         orientation.value.z - point.z};
      if (length(toward) > 0.0) {
        if (dot(normals[index], toward) < 0.0) {
          normals[index] = negate(normals[index]);
        }
        orientation_valid[index] = 1U;
      }
    }
    oriented_count += orientation_valid[index];
  });

  std::vector<Vec3f> points(surface.points().begin(), surface.points().end());
  auto output = OwnedSurface::create(std::move(points), std::move(normals), std::move(output_mask),
                                     surface.grid(), surface.unit(), surface.frame());
  if (!output) {
    return Result<NormalBoundaryResult>::failure(std::move(output).error());
  }
  std::vector<std::uint8_t> boundary;
  if (surface.grid()) {
    boundary = detect_grid_boundaries(output.value().view(), boundary_radius_mm);
  } else {
    auto detected = backends::pcl_backend::detect_unorganized_boundaries(output.value().view(),
                                                                         boundary_radius_mm);
    if (!detected) {
      return Result<NormalBoundaryResult>::failure(std::move(detected).error());
    }
    boundary = std::move(detected).value();
  }
  const bool orientation_proven = valid_normal_count > 0U && oriented_count == valid_normal_count;
  return Result<NormalBoundaryResult>::success(
      NormalBoundaryResult(std::move(output).value(), std::move(reasons),
                           std::move(orientation_valid), std::move(boundary), orientation_proven));
}

} // namespace

Result<NormalBoundaryResult>
prepare_normals_and_boundaries(SurfaceView surface, double normal_radius_mm,
                               std::uint32_t normal_min_neighbors, double boundary_radius_mm,
                               NormalOrientationHint orientation) noexcept {
  try {
    return prepare_impl(surface, normal_radius_mm, normal_min_neighbors, boundary_radius_mm,
                        orientation);
  } catch (const std::exception& exception) {
    return Result<NormalBoundaryResult>::failure(
        normal_error(ErrorCode::internal_error, "exception", exception.what()));
  } catch (...) {
    return Result<NormalBoundaryResult>::failure(
        normal_error(ErrorCode::internal_error, "exception", "unknown exception"));
  }
}

} // namespace pointcloud_ad::preprocess
