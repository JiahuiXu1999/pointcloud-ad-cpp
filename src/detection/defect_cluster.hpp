#pragma once

#include "deviation_field.hpp"

#include <cstddef>
#include <cstdint>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/surface.hpp>
#include <vector>

namespace pointcloud_ad::detection {

// Kind of surface deviation a cluster represents. Missing material is classified by a separate
// coverage pass and is not produced here.
enum class DefectType : std::uint8_t { dent, bump };

struct DefectCluster final {
  DefectType type{DefectType::dent};
  // Storage indices into the aligned scan for the points forming this cluster.
  std::vector<std::size_t> point_indices;
};

// Classifies trustworthy deviation samples into bump and dent candidates by signed deviation
// threshold, clusters each class separately with 3D Euclidean clustering, and drops clusters below
// the minimum point count. Cluster membership is deterministic.
[[nodiscard]] Result<std::vector<DefectCluster>>
cluster_deviation_defects(SurfaceView aligned_scan, const comparison::DeviationField& field,
                          const ValidatedDetectionConfig& config) noexcept;

} // namespace pointcloud_ad::detection
