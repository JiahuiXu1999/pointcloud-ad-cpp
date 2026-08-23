#pragma once

#include <cstddef>
#include <cstdint>
#include <pointcloud_ad/surface.hpp>
#include <string_view>

namespace pointcloud_ad::io {

enum class PcdEncoding : std::uint8_t { binary, ascii };

[[nodiscard]] Result<OwnedSurface> read_pcd(std::string_view path_utf8, LengthUnit unit,
                                            FrameId frame) noexcept;

[[nodiscard]] Result<std::size_t> write_pcd(std::string_view path_utf8, SurfaceView surface,
                                            PcdEncoding encoding = PcdEncoding::binary) noexcept;

} // namespace pointcloud_ad::io
