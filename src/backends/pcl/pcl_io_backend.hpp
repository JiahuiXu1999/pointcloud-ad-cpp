#pragma once

#include <cstddef>
#include <pointcloud_ad/surface.hpp>
#include <string_view>

namespace pointcloud_ad::backends::pcl_backend {

[[nodiscard]] Result<OwnedSurface> read_ply_file(std::string_view path_utf8, LengthUnit unit,
                                                 FrameId frame) noexcept;

[[nodiscard]] Result<std::size_t> write_ply_file(std::string_view path_utf8, SurfaceView surface,
                                                 bool binary) noexcept;

[[nodiscard]] Result<OwnedSurface> read_pcd_file(std::string_view path_utf8, LengthUnit unit,
                                                 FrameId frame) noexcept;

[[nodiscard]] Result<std::size_t> write_pcd_file(std::string_view path_utf8, SurfaceView surface,
                                                 bool binary) noexcept;

} // namespace pointcloud_ad::backends::pcl_backend
