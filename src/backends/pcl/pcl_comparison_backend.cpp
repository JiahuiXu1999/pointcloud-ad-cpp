#include "pcl_comparison_backend.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_ad::backends::pcl_backend {
namespace {

[[nodiscard]] Error comparison_error(std::string reason) {
  return Error{ErrorCode::internal_error,
               PipelineStage::compare,
               "PCL nearest-neighbour search failed",
               {{"reason", std::move(reason)}}};
}

[[nodiscard]] bool valid_at(SurfaceView surface, std::size_t index) noexcept {
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

} // namespace

Result<NearestNeighborResult> nearest_neighbors(SurfaceView reference, SurfaceView query,
                                                double max_distance_mm) noexcept {
  try {
    auto reference_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::vector<std::size_t> storage_indices;
    for_each_logical_index(reference, [&](std::size_t index) {
      if (!valid_at(reference, index)) {
        return;
      }
      const auto point = reference.points()[index];
      reference_cloud->points.emplace_back(point.x, point.y, point.z);
      storage_indices.push_back(index);
    });
    reference_cloud->width = static_cast<std::uint32_t>(reference_cloud->points.size());
    reference_cloud->height = 1U;
    reference_cloud->is_dense = true;

    NearestNeighborResult result;
    result.neighbor_index.assign(query.storage_size(), -1);
    result.distance_mm.assign(query.storage_size(), 0.0F);

    if (reference_cloud->empty()) {
      return Result<NearestNeighborResult>::success(std::move(result));
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(reference_cloud);

    pcl::PointXYZ search_point;
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    const float squared_limit = static_cast<float>(max_distance_mm * max_distance_mm);

    for_each_logical_index(query, [&](std::size_t index) {
      if (!valid_at(query, index)) {
        return;
      }
      const auto point = query.points()[index];
      search_point.x = point.x;
      search_point.y = point.y;
      search_point.z = point.z;
      if (tree.nearestKSearch(search_point, 1, indices, squared_distances) == 0) {
        return;
      }
      if (squared_distances[0] > squared_limit) {
        return;
      }
      result.neighbor_index[index] = static_cast<std::int32_t>(storage_indices[indices[0]]);
      result.distance_mm[index] = std::sqrt(squared_distances[0]);
    });

    return Result<NearestNeighborResult>::success(std::move(result));
  } catch (const std::exception& exception) {
    return Result<NearestNeighborResult>::failure(comparison_error(exception.what()));
  } catch (...) {
    return Result<NearestNeighborResult>::failure(comparison_error("unknown backend exception"));
  }
}

} // namespace pointcloud_ad::backends::pcl_backend
