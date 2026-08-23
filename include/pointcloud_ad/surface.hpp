#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <pointcloud_ad/geometry.hpp>
#include <span>
#include <utility>
#include <vector>

namespace pointcloud_ad {

struct GridTopology final {
  std::uint32_t width{};
  std::uint32_t height{};
  std::uint32_t row_stride{}; // Elements, not bytes.

  friend bool operator==(const GridTopology&, const GridTopology&) = default;
};

namespace detail {

[[nodiscard]] inline Error surface_error(std::string field, std::string reason) {
  return Error{ErrorCode::invalid_input,
               PipelineStage::validate,
               "invalid surface contract",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

[[nodiscard]] inline bool is_finite(Vec3f point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] inline Result<std::size_t>
validate_surface_data(std::span<const Vec3f> points, std::span<const Vec3f> normals,
                      std::span<const std::uint8_t> valid,
                      const std::optional<GridTopology>& grid) {
  if (points.empty()) {
    return Result<std::size_t>::failure(surface_error("points", "must not be empty"));
  }
  if (!normals.empty() && normals.size() != points.size()) {
    return Result<std::size_t>::failure(
        surface_error("normals", "must be empty or match point storage size"));
  }
  if (!valid.empty() && valid.size() != points.size()) {
    return Result<std::size_t>::failure(
        surface_error("valid", "must be empty or match point storage size"));
  }

  std::size_t logical_size = points.size();
  if (grid) {
    if (grid->width == 0U || grid->height == 0U) {
      return Result<std::size_t>::failure(
          surface_error("grid", "width and height must be positive"));
    }
    if (grid->row_stride < grid->width) {
      return Result<std::size_t>::failure(
          surface_error("grid.row_stride", "must be at least grid.width"));
    }
    const auto maximum = std::numeric_limits<std::size_t>::max();
    if (static_cast<std::size_t>(grid->height - 1U) >
        (maximum - static_cast<std::size_t>(grid->width)) /
            static_cast<std::size_t>(grid->row_stride)) {
      return Result<std::size_t>::failure(surface_error("grid", "storage size overflows"));
    }
    const std::size_t required_storage =
        static_cast<std::size_t>(grid->height - 1U) * static_cast<std::size_t>(grid->row_stride) +
        static_cast<std::size_t>(grid->width);
    if (points.size() != required_storage) {
      return Result<std::size_t>::failure(
          surface_error("grid", "point storage does not match width, height, and row_stride"));
    }
    logical_size = static_cast<std::size_t>(grid->width) * static_cast<std::size_t>(grid->height);
  }

  const auto check_index = [&](std::size_t index) -> Result<std::size_t> {
    if (!valid.empty() && valid[index] > 1U) {
      return Result<std::size_t>::failure(surface_error("valid", "mask elements must be 0 or 1"));
    }
    const bool point_is_valid = valid.empty() || valid[index] == 1U;
    if (point_is_valid && !is_finite(points[index])) {
      return Result<std::size_t>::failure(
          surface_error("points", "valid points must contain only finite coordinates"));
    }
    if (point_is_valid && !normals.empty() && !is_finite(normals[index])) {
      return Result<std::size_t>::failure(
          surface_error("normals", "normals for valid points must be finite"));
    }
    return Result<std::size_t>::success(logical_size);
  };

  if (!grid) {
    for (std::size_t index = 0; index < points.size(); ++index) {
      auto checked = check_index(index);
      if (!checked) {
        return checked;
      }
    }
  } else {
    for (std::size_t row = 0; row < grid->height; ++row) {
      for (std::size_t column = 0; column < grid->width; ++column) {
        const std::size_t index = row * static_cast<std::size_t>(grid->row_stride) + column;
        auto checked = check_index(index);
        if (!checked) {
          return checked;
        }
      }
    }
  }
  return Result<std::size_t>::success(logical_size);
}

} // namespace detail

class OwnedSurface;

// Borrows all point arrays and its frame identifier. The caller keeps them alive for the full use
// of the view. Grid storage may contain padding between rows; size() reports logical points.
class SurfaceView final {
public:
  [[nodiscard]] static Result<SurfaceView> create(std::span<const Vec3f> points,
                                                  std::span<const Vec3f> normals,
                                                  std::span<const std::uint8_t> valid,
                                                  std::optional<GridTopology> grid, LengthUnit unit,
                                                  const FrameId& frame) {
    auto size = detail::validate_surface_data(points, normals, valid, grid);
    if (!size) {
      return Result<SurfaceView>::failure(std::move(size).error());
    }
    return Result<SurfaceView>::success(
        SurfaceView(points, normals, valid, std::move(grid), unit, frame, size.value()));
  }

  [[nodiscard]] std::span<const Vec3f> points() const noexcept {
    return points_;
  }
  [[nodiscard]] std::span<const Vec3f> normals() const noexcept {
    return normals_;
  }
  [[nodiscard]] std::span<const std::uint8_t> valid() const noexcept {
    return valid_;
  }
  [[nodiscard]] const std::optional<GridTopology>& grid() const noexcept {
    return grid_;
  }
  [[nodiscard]] LengthUnit unit() const noexcept {
    return unit_;
  }
  [[nodiscard]] const FrameId& frame() const noexcept {
    return *frame_;
  }
  [[nodiscard]] std::size_t size() const noexcept {
    return logical_size_;
  }
  [[nodiscard]] std::size_t storage_size() const noexcept {
    return points_.size();
  }

private:
  friend class OwnedSurface;

  SurfaceView(std::span<const Vec3f> points, std::span<const Vec3f> normals,
              std::span<const std::uint8_t> valid, std::optional<GridTopology> grid,
              LengthUnit unit, const FrameId& frame, std::size_t logical_size) noexcept
      : points_(points), normals_(normals), valid_(valid), grid_(std::move(grid)), unit_(unit),
        frame_(&frame), logical_size_(logical_size) {}

  std::span<const Vec3f> points_;
  std::span<const Vec3f> normals_;
  std::span<const std::uint8_t> valid_;
  std::optional<GridTopology> grid_;
  LengthUnit unit_;
  const FrameId* frame_;
  std::size_t logical_size_;
};

class OwnedSurface final {
public:
  OwnedSurface(const OwnedSurface&) = delete;
  OwnedSurface& operator=(const OwnedSurface&) = delete;
  OwnedSurface(OwnedSurface&&) noexcept = default;
  OwnedSurface& operator=(OwnedSurface&&) noexcept = default;
  ~OwnedSurface() = default;

  [[nodiscard]] static Result<OwnedSurface>
  create(std::vector<Vec3f> points, std::vector<Vec3f> normals, std::vector<std::uint8_t> valid,
         std::optional<GridTopology> grid, LengthUnit unit, FrameId frame) {
    auto size = detail::validate_surface_data(points, normals, valid, grid);
    if (!size) {
      return Result<OwnedSurface>::failure(std::move(size).error());
    }
    return Result<OwnedSurface>::success(OwnedSurface(std::move(points), std::move(normals),
                                                      std::move(valid), std::move(grid), unit,
                                                      std::move(frame), size.value()));
  }

  [[nodiscard]] SurfaceView view() const noexcept {
    return SurfaceView(points_, normals_, valid_, grid_, unit_, frame_, logical_size_);
  }
  [[nodiscard]] std::size_t size() const noexcept {
    return logical_size_;
  }
  [[nodiscard]] std::size_t storage_size() const noexcept {
    return points_.size();
  }

private:
  OwnedSurface(std::vector<Vec3f> points, std::vector<Vec3f> normals,
               std::vector<std::uint8_t> valid, std::optional<GridTopology> grid, LengthUnit unit,
               FrameId frame, std::size_t logical_size)
      : points_(std::move(points)), normals_(std::move(normals)), valid_(std::move(valid)),
        grid_(std::move(grid)), unit_(unit), frame_(std::move(frame)), logical_size_(logical_size) {
  }

  std::vector<Vec3f> points_;
  std::vector<Vec3f> normals_;
  std::vector<std::uint8_t> valid_;
  std::optional<GridTopology> grid_;
  LengthUnit unit_;
  FrameId frame_;
  std::size_t logical_size_;
};

} // namespace pointcloud_ad
