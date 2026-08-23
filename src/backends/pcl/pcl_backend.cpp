#include "pcl_backend.hpp"

#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_ad::backends::pcl_backend {
namespace {

[[nodiscard]] Error backend_error(std::string reason) {
  return Error{ErrorCode::internal_error,
               PipelineStage::none,
               "PCL surface conversion failed",
               {{"reason", std::move(reason)}}};
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

[[nodiscard]] std::uint8_t mask_at(SurfaceView surface, std::size_t index) noexcept {
  return surface.valid().empty() ? std::uint8_t{1} : surface.valid()[index];
}

} // namespace

struct PclSurface::Impl final {
  pcl::PCLPointCloud2 cloud;
  std::vector<std::uint8_t> valid;
  bool has_explicit_mask{};
  bool has_normals{};
};

PclSurface::PclSurface(std::unique_ptr<Impl> implementation) noexcept
    : implementation_(std::move(implementation)) {}

PclSurface::PclSurface(PclSurface&&) noexcept = default;
PclSurface& PclSurface::operator=(PclSurface&&) noexcept = default;
PclSurface::~PclSurface() = default;

Result<PclSurface> to_pcl_surface(SurfaceView surface) noexcept {
  try {
    auto implementation = std::make_unique<PclSurface::Impl>();
    implementation->has_explicit_mask = !surface.valid().empty();
    implementation->has_normals = !surface.normals().empty();
    implementation->valid.reserve(surface.size());

    const std::uint32_t width =
        surface.grid() ? surface.grid()->width : static_cast<std::uint32_t>(surface.size());
    const std::uint32_t height = surface.grid() ? surface.grid()->height : 1U;
    bool dense = true;

    if (implementation->has_normals) {
      pcl::PointCloud<pcl::PointNormal> cloud;
      cloud.points.reserve(surface.size());
      for_each_logical_index(surface, [&](std::size_t index) {
        const auto point = surface.points()[index];
        const auto normal = surface.normals()[index];
        cloud.points.emplace_back(point.x, point.y, point.z, normal.x, normal.y, normal.z);
        const auto valid = mask_at(surface, index);
        implementation->valid.push_back(valid);
        dense = dense && valid == 1U;
      });
      cloud.width = width;
      cloud.height = height;
      cloud.is_dense = dense;
      pcl::toPCLPointCloud2(cloud, implementation->cloud);
    } else {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.points.reserve(surface.size());
      for_each_logical_index(surface, [&](std::size_t index) {
        const auto point = surface.points()[index];
        cloud.points.emplace_back(point.x, point.y, point.z);
        const auto valid = mask_at(surface, index);
        implementation->valid.push_back(valid);
        dense = dense && valid == 1U;
      });
      cloud.width = width;
      cloud.height = height;
      cloud.is_dense = dense;
      pcl::toPCLPointCloud2(cloud, implementation->cloud);
    }

    return Result<PclSurface>::success(PclSurface(std::move(implementation)));
  } catch (const std::exception& exception) {
    return Result<PclSurface>::failure(backend_error(exception.what()));
  } catch (...) {
    return Result<PclSurface>::failure(backend_error("unknown backend exception"));
  }
}

Result<OwnedSurface> from_pcl_surface(const PclSurface& surface, LengthUnit unit,
                                      FrameId frame) noexcept {
  try {
    if (!surface.implementation_) {
      return Result<OwnedSurface>::failure(backend_error("backend surface was moved from"));
    }
    const auto& implementation = *surface.implementation_;
    const auto count = static_cast<std::size_t>(implementation.cloud.width) *
                       static_cast<std::size_t>(implementation.cloud.height);
    if (count == 0U || count != implementation.valid.size()) {
      return Result<OwnedSurface>::failure(backend_error("backend point count is inconsistent"));
    }

    std::vector<Vec3f> points;
    points.reserve(count);
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromPCLPointCloud2(implementation.cloud, point_cloud);
    if (point_cloud.size() != count) {
      return Result<OwnedSurface>::failure(backend_error("XYZ conversion changed point count"));
    }
    for (const auto& point : point_cloud.points) {
      points.push_back(Vec3f{point.x, point.y, point.z});
    }

    std::vector<Vec3f> normals;
    if (implementation.has_normals) {
      pcl::PointCloud<pcl::Normal> normal_cloud;
      pcl::fromPCLPointCloud2(implementation.cloud, normal_cloud);
      if (normal_cloud.size() != count) {
        return Result<OwnedSurface>::failure(
            backend_error("normal conversion changed point count"));
      }
      normals.reserve(count);
      for (const auto& normal : normal_cloud.points) {
        normals.push_back(Vec3f{normal.normal_x, normal.normal_y, normal.normal_z});
      }
    }

    std::optional<GridTopology> grid;
    if (implementation.cloud.height > 1U) {
      grid = GridTopology{implementation.cloud.width, implementation.cloud.height,
                          implementation.cloud.width};
    }
    std::vector<std::uint8_t> valid =
        implementation.has_explicit_mask ? implementation.valid : std::vector<std::uint8_t>{};
    return OwnedSurface::create(std::move(points), std::move(normals), std::move(valid), grid, unit,
                                std::move(frame));
  } catch (const std::exception& exception) {
    return Result<OwnedSurface>::failure(backend_error(exception.what()));
  } catch (...) {
    return Result<OwnedSurface>::failure(backend_error("unknown backend exception"));
  }
}

} // namespace pointcloud_ad::backends::pcl_backend
