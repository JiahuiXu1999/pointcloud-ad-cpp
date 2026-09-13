#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <pointcloud_ad/geometry.hpp>
#include <span>
#include <stdexcept>
#include <vector>

namespace pointcloud_ad::backends::cuda_backend {

// Flat preorder BVH with median splits on the widest coordinate axis. An interior node has
// count=0 and its first child follows immediately; escape skips its whole subtree. Leaves
// reference contiguous packed points. Bounds are float input extrema (no quantization).
struct SpatialNode final {
  Vec3f lower;
  Vec3f upper;
  std::uint32_t begin{};
  std::uint32_t count{};
  std::uint32_t escape{};
  std::int32_t minimum_index{};
};

struct SpatialIndex final {
  std::vector<Vec3f> points;
  std::vector<std::int32_t> original_indices;
  std::vector<SpatialNode> nodes;
};

[[nodiscard]] inline bool finite_point(Vec3f point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

// Host-only construction, independent of the CUDA Toolkit. This permits CPU invariant tests
// even on hosts without a GPU. Total split ordering and sorted leaves make packing deterministic.
[[nodiscard]] inline SpatialIndex build_spatial_index(std::span<const Vec3f> reference) {
  if (reference.empty() ||
      reference.size() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
    throw std::invalid_argument("spatial index requires a nonempty int32-sized reference");
  }
  for (const auto point : reference) {
    if (!finite_point(point)) {
      throw std::invalid_argument("spatial index requires finite coordinates");
    }
  }
  SpatialIndex result;
  result.original_indices.resize(reference.size());
  std::iota(result.original_indices.begin(), result.original_indices.end(), std::int32_t{0});
  result.nodes.reserve(reference.size() / 4U + 1U);
  constexpr std::uint32_t leaf_capacity = 16U;
  const auto coordinate = [](Vec3f point, unsigned int axis) {
    return axis == 0U ? point.x : (axis == 1U ? point.y : point.z);
  };
  const auto build = [&](auto&& self, std::uint32_t begin, std::uint32_t end) -> void {
    const auto node_index = static_cast<std::uint32_t>(result.nodes.size());
    result.nodes.emplace_back();
    SpatialNode node;
    node.lower = node.upper = reference[static_cast<std::size_t>(result.original_indices[begin])];
    node.begin = begin;
    node.count = end - begin;
    node.minimum_index = std::numeric_limits<std::int32_t>::max();
    for (auto i = begin; i < end; ++i) {
      const auto original = result.original_indices[i];
      const auto point = reference[static_cast<std::size_t>(original)];
      node.lower.x = std::min(node.lower.x, point.x);
      node.lower.y = std::min(node.lower.y, point.y);
      node.lower.z = std::min(node.lower.z, point.z);
      node.upper.x = std::max(node.upper.x, point.x);
      node.upper.y = std::max(node.upper.y, point.y);
      node.upper.z = std::max(node.upper.z, point.z);
      node.minimum_index = std::min(node.minimum_index, original);
    }
    auto first = result.original_indices.begin() + begin;
    auto last = result.original_indices.begin() + end;
    if (node.count <= leaf_capacity) {
      std::sort(first, last);
    } else {
      unsigned int axis = 0U;
      double extent = static_cast<double>(node.upper.x) - node.lower.x;
      for (unsigned int candidate = 1U; candidate < 3U; ++candidate) {
        const double candidate_extent = static_cast<double>(coordinate(node.upper, candidate)) -
                                        coordinate(node.lower, candidate);
        if (candidate_extent > extent) {
          axis = candidate;
          extent = candidate_extent;
        }
      }
      const auto middle = begin + node.count / 2U;
      std::nth_element(first, result.original_indices.begin() + middle, last,
                       [&](std::int32_t left, std::int32_t right) {
                         const auto l = coordinate(reference[static_cast<std::size_t>(left)], axis);
                         const auto r =
                             coordinate(reference[static_cast<std::size_t>(right)], axis);
                         return l < r || (l == r && left < right);
                       });
      node.count = 0U;
      self(self, begin, middle);
      self(self, middle, end);
    }
    node.escape = static_cast<std::uint32_t>(result.nodes.size());
    result.nodes[node_index] = node;
  };
  build(build, 0U, static_cast<std::uint32_t>(reference.size()));
  result.points.reserve(reference.size());
  for (const auto index : result.original_indices) {
    result.points.push_back(reference[static_cast<std::size_t>(index)]);
  }
  return result;
}

} // namespace pointcloud_ad::backends::cuda_backend
