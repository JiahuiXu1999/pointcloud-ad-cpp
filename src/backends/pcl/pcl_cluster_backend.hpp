#pragma once

#include <cstddef>
#include <cstdint>
#include <pointcloud_ad/surface.hpp>
#include <span>
#include <vector>

namespace pointcloud_ad::backends::pcl_backend {

// Labels candidate points into 3D Euclidean clusters. The returned vector is parallel to
// `candidate_indices`; entry i is the zero-based cluster id of candidate i (deterministic, assigned
// in candidate order). Two candidates join the same cluster when a chain of links of length at most
// `tolerance_mm` connects them. PCL types never leave this translation unit.
[[nodiscard]] Result<std::vector<std::int32_t>>
euclidean_cluster(SurfaceView surface, std::span<const std::size_t> candidate_indices,
                  double tolerance_mm) noexcept;

} // namespace pointcloud_ad::backends::pcl_backend
