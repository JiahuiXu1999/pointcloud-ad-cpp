#include "pcl_preprocess_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_ad::backends::pcl_backend {
namespace {

struct CloudMapping final {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::vector<std::size_t> storage_indices;
};

[[nodiscard]] Error feature_error(std::string operation, std::string reason) {
  return Error{ErrorCode::internal_error,
               PipelineStage::preprocess,
               "PCL preprocessing failed",
               {{"operation", std::move(operation)}, {"reason", std::move(reason)}}};
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

[[nodiscard]] CloudMapping make_valid_cloud(SurfaceView surface) {
  CloudMapping mapping{std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(), {}};
  mapping.cloud->points.reserve(surface.size());
  mapping.storage_indices.reserve(surface.size());
  for_each_logical_index(surface, [&](std::size_t index) {
    if (!surface.valid().empty() && surface.valid()[index] == 0U) {
      return;
    }
    const auto point = surface.points()[index];
    mapping.cloud->points.emplace_back(point.x, point.y, point.z);
    mapping.storage_indices.push_back(index);
  });
  mapping.cloud->width = static_cast<std::uint32_t>(mapping.cloud->size());
  mapping.cloud->height = 1U;
  mapping.cloud->is_dense = true;
  return mapping;
}

[[nodiscard]] Vec3f normalized(Vec3f value) noexcept {
  const double length =
      std::sqrt(static_cast<double>(value.x) * value.x + static_cast<double>(value.y) * value.y +
                static_cast<double>(value.z) * value.z);
  if (!(length > 0.0) || !std::isfinite(length)) {
    return Vec3f{};
  }
  return Vec3f{static_cast<float>(value.x / length), static_cast<float>(value.y / length),
               static_cast<float>(value.z / length)};
}

[[nodiscard]] Vec3f cross(Vec3f left, Vec3f right) noexcept {
  return Vec3f{left.y * right.z - left.z * right.y, left.z * right.x - left.x * right.z,
               left.x * right.y - left.y * right.x};
}

[[nodiscard]] double dot(Vec3f left, Vec3f right) noexcept {
  return static_cast<double>(left.x) * right.x + static_cast<double>(left.y) * right.y +
         static_cast<double>(left.z) * right.z;
}

} // namespace

Result<EstimatedNormals> estimate_normals(SurfaceView surface, double radius_mm,
                                          std::uint32_t minimum_neighbors) noexcept {
  try {
    auto mapping = make_valid_cloud(surface);
    EstimatedNormals result{std::vector<Vec3f>(surface.storage_size()),
                            std::vector<std::uint8_t>(surface.storage_size(), 0U)};
    if (mapping.cloud->empty()) {
      return Result<EstimatedNormals>::success(std::move(result));
    }
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(mapping.cloud);
    pcl::Indices neighbors;
    std::vector<float> squared_distances;
    for (std::size_t cloud_index = 0; cloud_index < mapping.cloud->size(); ++cloud_index) {
      neighbors.clear();
      squared_distances.clear();
      tree->radiusSearch(static_cast<int>(cloud_index), radius_mm, neighbors, squared_distances);
      std::sort(neighbors.begin(), neighbors.end());
      if (neighbors.size() < minimum_neighbors || neighbors.size() < 3U) {
        continue;
      }
      Eigen::Vector4f plane;
      float curvature = std::numeric_limits<float>::quiet_NaN();
      if (!pcl::computePointNormal(*mapping.cloud, neighbors, plane, curvature)) {
        continue;
      }
      const Vec3f normal = normalized(Vec3f{plane.x(), plane.y(), plane.z()});
      if (normal.x == 0.0F && normal.y == 0.0F && normal.z == 0.0F) {
        continue;
      }
      const auto storage_index = mapping.storage_indices[cloud_index];
      result.normals[storage_index] = normal;
      result.valid[storage_index] = 1U;
    }
    return Result<EstimatedNormals>::success(std::move(result));
  } catch (const std::exception& exception) {
    return Result<EstimatedNormals>::failure(feature_error("normal_estimation", exception.what()));
  } catch (...) {
    return Result<EstimatedNormals>::failure(
        feature_error("normal_estimation", "unknown exception"));
  }
}

Result<std::vector<std::uint8_t>> detect_unorganized_boundaries(SurfaceView surface,
                                                                double radius_mm) noexcept {
  try {
    auto mapping = make_valid_cloud(surface);
    std::vector<std::uint8_t> boundaries(surface.storage_size(), 0U);
    if (mapping.cloud->empty()) {
      return Result<std::vector<std::uint8_t>>::success(std::move(boundaries));
    }
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(mapping.cloud);
    pcl::Indices neighbors;
    std::vector<float> squared_distances;
    constexpr double pi = 3.14159265358979323846;
    constexpr double threshold = pi / 2.0;
    for (std::size_t cloud_index = 0; cloud_index < mapping.cloud->size(); ++cloud_index) {
      neighbors.clear();
      squared_distances.clear();
      tree->radiusSearch(static_cast<int>(cloud_index), radius_mm, neighbors, squared_distances);
      const auto storage_index = mapping.storage_indices[cloud_index];
      const auto normal = normalized(surface.normals()[storage_index]);
      const Vec3f axis =
          std::abs(normal.x) <= std::abs(normal.y) && std::abs(normal.x) <= std::abs(normal.z)
              ? Vec3f{1.0F, 0.0F, 0.0F}
              : (std::abs(normal.y) <= std::abs(normal.z) ? Vec3f{0.0F, 1.0F, 0.0F}
                                                          : Vec3f{0.0F, 0.0F, 1.0F});
      const auto u = normalized(cross(normal, axis));
      const auto v = cross(normal, u);
      std::vector<double> angles;
      angles.reserve(neighbors.size());
      const auto origin = surface.points()[storage_index];
      for (const auto neighbor : neighbors) {
        if (neighbor == static_cast<int>(cloud_index)) {
          continue;
        }
        const auto neighbor_point = surface.points()[mapping.storage_indices[neighbor]];
        const Vec3f delta{neighbor_point.x - origin.x, neighbor_point.y - origin.y,
                          neighbor_point.z - origin.z};
        const double projected_x = dot(delta, u);
        const double projected_y = dot(delta, v);
        if (projected_x * projected_x + projected_y * projected_y > 1.0e-20) {
          angles.push_back(std::atan2(projected_y, projected_x));
        }
      }
      if (angles.size() < 3U) {
        boundaries[storage_index] = 1U;
        continue;
      }
      std::sort(angles.begin(), angles.end());
      double maximum_gap = angles.front() + 2.0 * pi - angles.back();
      for (std::size_t index = 1; index < angles.size(); ++index) {
        maximum_gap = std::max(maximum_gap, angles[index] - angles[index - 1U]);
      }
      boundaries[storage_index] = maximum_gap > threshold ? 1U : 0U;
    }
    return Result<std::vector<std::uint8_t>>::success(std::move(boundaries));
  } catch (const std::exception& exception) {
    return Result<std::vector<std::uint8_t>>::failure(
        feature_error("boundary_detection", exception.what()));
  } catch (...) {
    return Result<std::vector<std::uint8_t>>::failure(
        feature_error("boundary_detection", "unknown exception"));
  }
}

} // namespace pointcloud_ad::backends::pcl_backend
