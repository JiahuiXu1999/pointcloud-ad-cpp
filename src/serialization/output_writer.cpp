#include "output_writer.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pointcloud_ad::serialization {
namespace {

[[nodiscard]] Error output_error(ErrorCode code, std::string message) {
  return Error{code, PipelineStage::serialization, std::move(message), {}};
}

// --- SHA-256 -----------------------------------------------------------------

constexpr std::array<std::uint32_t, 64> kSha256Constants{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U,
    0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
    0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U,
    0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
    0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
    0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU,
    0x5b9cca4fU, 0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

[[nodiscard]] std::uint32_t rotate_right(std::uint32_t value, std::uint32_t count) noexcept {
  return (value >> count) | (value << (32U - count));
}

[[nodiscard]] std::array<std::uint32_t, 8> compress(const std::array<std::uint32_t, 8>& state,
                                                    const std::array<std::uint32_t, 64>& schedule,
                                                    const std::array<std::uint32_t, 8>& hash) {
  std::array<std::uint32_t, 8> working = hash;
  for (std::size_t index = 0; index < 64; ++index) {
    const std::uint32_t sum1 = rotate_right(working[4], 6) ^ rotate_right(working[4], 11) ^
                               rotate_right(working[4], 25);
    const std::uint32_t choose = (working[4] & working[5]) ^ (~working[4] & working[6]);
    const std::uint32_t temp1 =
        working[7] + sum1 + choose + kSha256Constants[index] + schedule[index];
    const std::uint32_t sum0 = rotate_right(working[0], 2) ^ rotate_right(working[0], 13) ^
                               rotate_right(working[0], 22);
    const std::uint32_t majority = (working[0] & working[1]) ^ (working[0] & working[2]) ^
                                   (working[1] & working[2]);
    const std::uint32_t temp2 = sum0 + majority;
    working[7] = working[6];
    working[6] = working[5];
    working[5] = working[4];
    working[4] = working[3] + temp1;
    working[3] = working[2];
    working[2] = working[1];
    working[1] = working[0];
    working[0] = temp1 + temp2;
  }
  std::array<std::uint32_t, 8> result{};
  for (std::size_t index = 0; index < 8; ++index) {
    result[index] = hash[index] + working[index];
  }
  (void)state;
  return result;
}

[[nodiscard]] std::string to_hex(const std::array<std::uint32_t, 8>& hash) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::uint32_t word : hash) {
    stream << std::setw(8) << word;
  }
  return stream.str();
}

} // namespace

std::string sha256_hex(std::string_view data) noexcept {
  try {
    std::array<std::uint32_t, 8> hash{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                                      0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};

    std::vector<std::uint8_t> bytes;
    bytes.reserve(data.size() + 72U);
    bytes.insert(bytes.end(), data.begin(), data.end());
    const std::uint64_t bit_length = static_cast<std::uint64_t>(data.size()) * 8U;
    bytes.push_back(0x80U);
    while (bytes.size() % 64U != 56U) {
      bytes.push_back(0x00U);
    }
    for (int shift = 56; shift >= 0; shift -= 8) {
      bytes.push_back(static_cast<std::uint8_t>((bit_length >> shift) & 0xFFU));
    }

    for (std::size_t offset = 0; offset < bytes.size(); offset += 64U) {
      std::array<std::uint32_t, 64> schedule{};
      for (std::size_t index = 0; index < 16; ++index) {
        schedule[index] = (static_cast<std::uint32_t>(bytes[offset + index * 4U]) << 24U) |
                          (static_cast<std::uint32_t>(bytes[offset + index * 4U + 1U]) << 16U) |
                          (static_cast<std::uint32_t>(bytes[offset + index * 4U + 2U]) << 8U) |
                          static_cast<std::uint32_t>(bytes[offset + index * 4U + 3U]);
      }
      for (std::size_t index = 16; index < 64; ++index) {
        const std::uint32_t s0 = rotate_right(schedule[index - 15], 7) ^
                                 rotate_right(schedule[index - 15], 18) ^
                                 (schedule[index - 15] >> 3);
        const std::uint32_t s1 = rotate_right(schedule[index - 2], 17) ^
                                 rotate_right(schedule[index - 2], 19) ^
                                 (schedule[index - 2] >> 10);
        schedule[index] = schedule[index - 16] + s0 + schedule[index - 7] + s1;
      }
      hash = compress({}, schedule, hash);
    }
    return to_hex(hash);
  } catch (...) {
    return {};
  }
}

Result<std::size_t> write_attribute_ply(std::string_view path_utf8, SurfaceView surface,
                                       std::span<const double> scalar,
                                       std::span<const std::uint8_t> reason_code,
                                       std::span<const std::int32_t> region_id) noexcept {
  try {
    if (scalar.size() != surface.storage_size() || reason_code.size() != surface.storage_size() ||
        region_id.size() != surface.storage_size()) {
      return Result<std::size_t>::failure(
          output_error(ErrorCode::invalid_input, "attribute arrays must span the surface storage"));
    }

    std::vector<std::size_t> indices;
    if (!surface.grid()) {
      for (std::size_t index = 0; index < surface.storage_size(); ++index) {
        indices.push_back(index);
      }
    } else {
      const auto& grid = *surface.grid();
      for (std::size_t row = 0; row < grid.height; ++row) {
        for (std::size_t column = 0; column < grid.width; ++column) {
          indices.push_back(row * static_cast<std::size_t>(grid.row_stride) + column);
        }
      }
    }

    std::ofstream file(std::string(path_utf8), std::ios::out | std::ios::trunc);
    if (!file) {
      return Result<std::size_t>::failure(
          output_error(ErrorCode::io_error, "failed to open output file"));
    }
    file << "ply\nformat ascii 1.0\nelement vertex " << indices.size()
         << "\nproperty float x\nproperty float y\nproperty float z\nproperty float scalar\n"
            "property uchar reason\nproperty int region_id\nend_header\n";
    file << std::fixed << std::setprecision(6);
    for (const std::size_t index : indices) {
      const Vec3f point = surface.points()[index];
      file << point.x << ' ' << point.y << ' ' << point.z << ' ' << scalar[index] << ' '
           << static_cast<unsigned int>(reason_code[index]) << ' ' << region_id[index] << '\n';
    }
    if (!file) {
      return Result<std::size_t>::failure(
          output_error(ErrorCode::io_error, "failed while writing output file"));
    }
    return Result<std::size_t>::success(indices.size());
  } catch (const std::exception& exception) {
    return Result<std::size_t>::failure(output_error(ErrorCode::internal_error, exception.what()));
  } catch (...) {
    return Result<std::size_t>::failure(
        output_error(ErrorCode::internal_error, "unknown exception"));
  }
}

Result<std::string> build_manifest(std::string_view schema_version, std::string_view generator,
                                   const std::vector<ManifestEntry>& entries) noexcept {
  try {
    nlohmann::json files = nlohmann::json::array();
    for (const auto& entry : entries) {
      files.push_back(nlohmann::json{{"path", entry.path}, {"sha256", entry.sha256}});
    }
    nlohmann::json document{{"schema_version", std::string(schema_version)},
                            {"generator", std::string(generator)},
                            {"files", std::move(files)}};
    return Result<std::string>::success(document.dump(2));
  } catch (const std::exception& exception) {
    return Result<std::string>::failure(
        output_error(ErrorCode::internal_error, exception.what()));
  } catch (...) {
    return Result<std::string>::failure(
        output_error(ErrorCode::internal_error, "unknown exception"));
  }
}

} // namespace pointcloud_ad::serialization
