#pragma once

#include <cstddef>
#include <cstdint>
#include <pointcloud_ad/surface.hpp>
#include <string_view>

namespace pointcloud_ad::io {

enum class PlyEncoding : std::uint8_t { binary_little_endian, ascii };

[[nodiscard]] Result<OwnedSurface> read_ply(std::string_view path_utf8, LengthUnit unit,
                                            FrameId frame) noexcept;

[[nodiscard]] Result<std::size_t>
write_ply(std::string_view path_utf8, SurfaceView surface,
          PlyEncoding encoding = PlyEncoding::binary_little_endian) noexcept;

} // namespace pointcloud_ad::io
