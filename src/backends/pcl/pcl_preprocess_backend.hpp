#pragma once

#include <cstdint>
#include <pointcloud_ad/surface.hpp>
#include <vector>

namespace pointcloud_ad::backends::pcl_backend {

struct EstimatedNormals final {
  std::vector<Vec3f> normals;
  std::vector<std::uint8_t> valid;
};

[[nodiscard]] Result<EstimatedNormals> estimate_normals(SurfaceView surface, double radius_mm,
                                                        std::uint32_t minimum_neighbors) noexcept;

[[nodiscard]] Result<std::vector<std::uint8_t>>
detect_unorganized_boundaries(SurfaceView surface, double radius_mm) noexcept;

} // namespace pointcloud_ad::backends::pcl_backend
