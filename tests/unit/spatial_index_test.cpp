#include "spatial_index.hpp"

#include <array>
#include <iostream>
#include <random>
#include <string_view>

namespace {
using namespace pointcloud_ad;
using namespace pointcloud_ad::backends::cuda_backend;
bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}
bool contains(const SpatialNode& node, Vec3f point) {
  return point.x >= node.lower.x && point.x <= node.upper.x && point.y >= node.lower.y &&
         point.y <= node.upper.y && point.z >= node.lower.z && point.z <= node.upper.z;
}
bool verify(const std::vector<Vec3f>& points) {
  const auto index = build_spatial_index(points);
  const auto repeated = build_spatial_index(points);
  bool passed = expect(index.original_indices == repeated.original_indices &&
                           index.nodes.size() == repeated.nodes.size(),
                       "deterministic layout");
  auto permutation = index.original_indices;
  std::sort(permutation.begin(), permutation.end());
  passed &= expect(index.points.size() == points.size() && permutation.size() == points.size(),
                   "storage size");
  for (std::size_t i = 0; i < points.size(); ++i) {
    passed &= expect(permutation[i] == static_cast<std::int32_t>(i), "indices form a permutation");
    const auto p = points[static_cast<std::size_t>(index.original_indices[i])];
    passed &=
        expect(index.points[i].x == p.x && index.points[i].y == p.y && index.points[i].z == p.z,
               "packing preserves coordinates");
  }
  std::vector<unsigned int> visits(points.size(), 0U);
  for (std::size_t cursor = 0; cursor < index.nodes.size(); ++cursor) {
    const auto& node = index.nodes[cursor];
    const auto& repeat = repeated.nodes[cursor];
    passed &=
        expect(node.escape > cursor && node.escape <= index.nodes.size(), "forward bounded escape");
    passed &= expect(node.begin == repeat.begin && node.count == repeat.count &&
                         node.escape == repeat.escape && node.minimum_index == repeat.minimum_index,
                     "deterministic node metadata");
    if (node.count == 0U) {
      const auto left = cursor + 1U;
      if (!expect(left < index.nodes.size(), "interior has left child")) {
        return false;
      }
      const auto right = index.nodes[left].escape;
      if (!expect(right < index.nodes.size(), "interior has right child")) {
        return false;
      }
      passed &= expect(index.nodes[right].escape == node.escape, "children exactly span subtree");
      passed &= expect(
          contains(node, index.nodes[left].lower) && contains(node, index.nodes[left].upper) &&
              contains(node, index.nodes[right].lower) && contains(node, index.nodes[right].upper),
          "parent contains child bounds");
      passed &= expect(node.minimum_index == std::min(index.nodes[left].minimum_index,
                                                      index.nodes[right].minimum_index),
                       "subtree minimum original index");
    } else {
      passed &= expect(node.count <= 16U && node.escape == cursor + 1U &&
                           static_cast<std::size_t>(node.begin) + node.count <= points.size(),
                       "bounded leaf");
      for (auto i = node.begin; i < node.begin + node.count; ++i) {
        ++visits[i];
        passed &= expect(contains(node, index.points[i]), "leaf contains every point");
        passed &= expect(index.original_indices[i] >= node.minimum_index,
                         "leaf minimum bounds tie index");
      }
      passed &=
          expect(index.original_indices[node.begin] == node.minimum_index, "leaf order stable");
    }
  }
  passed &= expect(index.nodes.front().escape == index.nodes.size(), "root escape ends traversal");
  passed &= expect(
      std::all_of(visits.begin(), visits.end(), [](unsigned int count) { return count == 1U; }),
      "leaves cover each packed slot once");
  return passed;
}
} // namespace

int main() {
  bool passed = true;
  std::mt19937 generator(317U);
  for (std::size_t count : {1U, 15U, 16U, 17U, 33U, 1023U, 4097U}) {
    std::vector<Vec3f> points;
    for (std::size_t i = 0; i < count; ++i) {
      points.push_back({static_cast<float>(generator() % 1000U) - 500.0F,
                        static_cast<float>(generator() % 200U) - 100.0F,
                        static_cast<float>(generator() % 7U)});
    }
    passed &= verify(points);
    std::fill(points.begin(), points.end(), Vec3f{1, -2, 3});
    passed &= verify(points);
  }
  for (auto invalid :
       {std::vector<Vec3f>{}, std::vector<Vec3f>{{std::numeric_limits<float>::quiet_NaN(), 0, 0}},
        std::vector<Vec3f>{{0, std::numeric_limits<float>::infinity(), 0}}}) {
    bool rejected = false;
    try {
      (void)build_spatial_index(invalid);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    passed &= expect(rejected, "invalid reference rejected");
  }
  return passed ? 0 : 1;
}
