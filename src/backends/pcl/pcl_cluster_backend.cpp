#include "pcl_cluster_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pointcloud_ad::backends::pcl_backend {
namespace {

[[nodiscard]] Error cluster_error(std::string reason) {
  return Error{ErrorCode::internal_error,
               PipelineStage::detect,
               "PCL Euclidean clustering failed",
               {{"reason", std::move(reason)}}};
}

class UnionFind final {
public:
  explicit UnionFind(std::size_t size) : parent_(size), rank_(size, 0U) {
    for (std::size_t index = 0; index < size; ++index) {
      parent_[index] = index;
    }
  }

  std::size_t find(std::size_t value) noexcept {
    if (parent_[value] != value) {
      parent_[value] = find(parent_[value]);
    }
    return parent_[value];
  }

  void unite(std::size_t left, std::size_t right) noexcept {
    const std::size_t left_root = find(left);
    const std::size_t right_root = find(right);
    if (left_root == right_root) {
      return;
    }
    if (rank_[left_root] < rank_[right_root]) {
      parent_[left_root] = right_root;
    } else if (rank_[left_root] > rank_[right_root]) {
      parent_[right_root] = left_root;
    } else {
      parent_[right_root] = left_root;
      ++rank_[left_root];
    }
  }

private:
  std::vector<std::size_t> parent_;
  std::vector<std::size_t> rank_;
};

} // namespace

Result<std::vector<std::int32_t>> euclidean_cluster(SurfaceView surface,
                                                    std::span<const std::size_t> candidate_indices,
                                                    double tolerance_mm) noexcept {
  try {
    if (!std::isfinite(tolerance_mm) || tolerance_mm <= 0.0) {
      return Result<std::vector<std::int32_t>>::failure(
          cluster_error("cluster tolerance must be finite and positive"));
    }
    if (candidate_indices.empty()) {
      return Result<std::vector<std::int32_t>>::success({});
    }

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->points.reserve(candidate_indices.size());
    for (const std::size_t storage_index : candidate_indices) {
      const auto point = surface.points()[storage_index];
      cloud->points.emplace_back(point.x, point.y, point.z);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1U;
    cloud->is_dense = true;

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    UnionFind union_find(candidate_indices.size());
    pcl::PointXYZ query;
    std::vector<int> neighbor_indices;
    std::vector<float> squared_distances;
    for (std::size_t candidate = 0; candidate < candidate_indices.size(); ++candidate) {
      query.x = cloud->points[candidate].x;
      query.y = cloud->points[candidate].y;
      query.z = cloud->points[candidate].z;
      neighbor_indices.clear();
      squared_distances.clear();
      if (tree.radiusSearch(query, tolerance_mm, neighbor_indices, squared_distances) == 0) {
        continue;
      }
      for (const int neighbor : neighbor_indices) {
        union_find.unite(candidate, static_cast<std::size_t>(neighbor));
      }
    }

    std::vector<std::int32_t> labels(candidate_indices.size(), -1);
    std::unordered_map<std::size_t, std::int32_t> root_to_label;
    std::int32_t next_label = 0;
    for (std::size_t candidate = 0; candidate < candidate_indices.size(); ++candidate) {
      const std::size_t root = union_find.find(candidate);
      auto [iterator, inserted] = root_to_label.emplace(root, next_label);
      if (inserted) {
        ++next_label;
      }
      labels[candidate] = iterator->second;
    }

    return Result<std::vector<std::int32_t>>::success(std::move(labels));
  } catch (const std::exception& exception) {
    return Result<std::vector<std::int32_t>>::failure(cluster_error(exception.what()));
  } catch (...) {
    return Result<std::vector<std::int32_t>>::failure(cluster_error("unknown backend exception"));
  }
}

} // namespace pointcloud_ad::backends::pcl_backend
