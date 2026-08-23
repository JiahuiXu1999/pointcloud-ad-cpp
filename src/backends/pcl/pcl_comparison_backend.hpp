#pragma once

#include <cstdint>
#include <pointcloud_ad/surface.hpp>
#include <vector>

namespace pointcloud_ad::backends::pcl_backend {

// Nearest-neighbour results for every storage index of a query surface. Entries are parallel to
// the query's storage layout; query points that are invalid or have no neighbour within the search
// radius report `neighbor_index == -1`. Neighbor indices refer to the reference surface's storage
// layout, not a compacted cloud, so callers can look up normals or masks directly. Distances are
// Euclidean millimetres.
struct NearestNeighborResult final {
  std::vector<std::int32_t> neighbor_index;
  std::vector<float> distance_mm;
};

// Builds one KD-tree over the reference's valid points and queries every valid query point exactly
// once, in storage order, for determinism. PCL types never leave this translation unit.
[[nodiscard]] Result<NearestNeighborResult>
nearest_neighbors(SurfaceView reference, SurfaceView query, double max_distance_mm) noexcept;

} // namespace pointcloud_ad::backends::pcl_backend
