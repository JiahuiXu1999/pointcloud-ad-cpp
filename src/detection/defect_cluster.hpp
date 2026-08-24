#pragma once

#include "coverage_field.hpp"
#include "deviation_field.hpp"

#include <cstddef>
#include <cstdint>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/surface.hpp>
#include <vector>

namespace pointcloud_ad::detection {

// Kind of surface deviation a cluster represents.
enum class DefectType : std::uint8_t { dent, bump, missing_material };

struct DefectCluster final {
  DefectType type{DefectType::dent};
  // Storage indices into the owning surface (the aligned scan for dent/bump, the reference for
  // missing material) for the points forming this cluster.
  std::vector<std::size_t> point_indices;
};

// Classifies trustworthy deviation samples into bump and dent candidates by signed deviation
// threshold, clusters each class separately with 3D Euclidean clustering, and drops clusters below
// the minimum point count. Cluster membership is deterministic.
[[nodiscard]] Result<std::vector<DefectCluster>>
cluster_deviation_defects(SurfaceView aligned_scan, const comparison::DeviationField& field,
                          const ValidatedDetectionConfig& config) noexcept;

// Clusters reference points whose coverage reason is `no_neighbor` into missing-material regions
// with 3D Euclidean clustering, dropping clusters below the minimum point count.
[[nodiscard]] Result<std::vector<DefectCluster>>
cluster_missing_material(SurfaceView reference, const comparison::CoverageField& field,
                         const ValidatedDetectionConfig& config) noexcept;

} // namespace pointcloud_ad::detection
