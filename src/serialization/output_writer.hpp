#pragma once

#include <pointcloud_ad/result.hpp>
#include <pointcloud_ad/surface.hpp>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace pointcloud_ad::serialization {

// Computes the lowercase hexadecimal SHA-256 digest of the input bytes.
[[nodiscard]] std::string sha256_hex(std::string_view data) noexcept;

// Writes an ASCII PLY with one vertex per logical surface point, carrying a scalar attribute, a
// reason code, and a region id. `scalar`, `reason_code`, and `region_id` must span the surface's
// storage layout. Returns the number of vertices written.
[[nodiscard]] Result<std::size_t>
write_attribute_ply(std::string_view path_utf8, SurfaceView surface, std::span<const double> scalar,
                    std::span<const std::uint8_t> reason_code,
                    std::span<const std::int32_t> region_id) noexcept;

struct ManifestEntry final {
  std::string path;
  std::string sha256;
};

// Builds a run manifest document listing each output file with its SHA-256 digest.
[[nodiscard]] Result<std::string>
build_manifest(std::string_view schema_version, std::string_view generator,
               const std::vector<ManifestEntry>& entries) noexcept;

} // namespace pointcloud_ad::serialization
