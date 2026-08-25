#include "validity.hpp"
#include "voxel_sampling.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <optional>
#include <pointcloud_ad/normalization.hpp>
#include <pointcloud_ad/surface.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#if defined(_WIN32)
#define NOMINMAX
#include <Psapi.h>
#include <Windows.h>
#elif defined(__linux__) || defined(__APPLE__)
#include <sys/resource.h>
#endif

namespace {

using Clock = std::chrono::steady_clock;
using pointcloud_ad::FrameId;
using pointcloud_ad::LengthUnit;
using pointcloud_ad::normalize_surface;
using pointcloud_ad::OwnedSurface;
using pointcloud_ad::Vec3f;
using pointcloud_ad::preprocess::apply_validity_mask;
using pointcloud_ad::preprocess::voxel_sample;

constexpr std::size_t kSmallPointCount = 100'000U;
constexpr std::size_t kLargePointCount = 1'000'000U;
constexpr double kVoxelSizeMm = 1.0;
constexpr double kMaximumLargeTotalMs = 180'000.0;
constexpr double kMaximumScalingRatio = 25.0;
constexpr std::uint64_t kMaximumPeakRssBytes = 2ULL * 1024ULL * 1024ULL * 1024ULL;

struct BenchmarkResult final {
  std::size_t input_points{};
  std::size_t valid_points{};
  std::size_t sampled_points{};
  double normalize_ms{};
  double validity_ms{};
  double voxel_sampling_ms{};
  std::uint64_t peak_rss_bytes{};
  std::uint64_t checksum{};

  [[nodiscard]] double total_ms() const noexcept {
    return normalize_ms + validity_ms + voxel_sampling_ms;
  }
};

[[nodiscard]] double elapsed_ms(Clock::time_point start) noexcept {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

[[nodiscard]] std::uint64_t peak_rss_bytes() noexcept {
#if defined(_WIN32)
  PROCESS_MEMORY_COUNTERS counters{};
  if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) == 0) {
    return 0U;
  }
  return static_cast<std::uint64_t>(counters.PeakWorkingSetSize);
#elif defined(__linux__) || defined(__APPLE__)
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    return 0U;
  }
#if defined(__APPLE__)
  return static_cast<std::uint64_t>(usage.ru_maxrss);
#else
  return static_cast<std::uint64_t>(usage.ru_maxrss) * 1024ULL;
#endif
#else
  return 0U;
#endif
}

[[nodiscard]] std::uint64_t hash_value(std::uint64_t hash, std::uint64_t value) noexcept {
  constexpr std::uint64_t prime = 1099511628211ULL;
  for (int byte = 0; byte < 8; ++byte) {
    hash ^= (value >> (byte * 8)) & 0xffULL;
    hash *= prime;
  }
  return hash;
}

[[nodiscard]] std::uint64_t result_checksum(pointcloud_ad::SurfaceView surface,
                                            std::size_t valid_points) noexcept {
  std::uint64_t hash = 14695981039346656037ULL;
  hash = hash_value(hash, valid_points);
  hash = hash_value(hash, surface.size());
  for (const auto point : surface.points()) {
    hash = hash_value(hash, std::bit_cast<std::uint32_t>(point.x));
    hash = hash_value(hash, std::bit_cast<std::uint32_t>(point.y));
    hash = hash_value(hash, std::bit_cast<std::uint32_t>(point.z));
  }
  return hash;
}

[[nodiscard]] OwnedSurface make_surface(std::size_t point_count, const FrameId& frame) {
  const auto width = static_cast<std::size_t>(std::ceil(std::sqrt(point_count)));
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  std::vector<std::uint8_t> valid;
  points.reserve(point_count);
  normals.reserve(point_count);
  valid.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    const auto column = index % width;
    const auto row = index / width;
    points.push_back(Vec3f{static_cast<float>(column) * 0.1F, static_cast<float>(row) * 0.1F,
                           static_cast<float>(index % 17U) * 0.001F});
    normals.push_back(Vec3f{0.0F, 0.0F, 1.0F});
    valid.push_back(1U);
  }
  return OwnedSurface::create(std::move(points), std::move(normals), std::move(valid), std::nullopt,
                              LengthUnit::millimeter, frame)
      .value();
}

[[nodiscard]] BenchmarkResult run_case(std::size_t point_count, const FrameId& frame) {
  auto input = make_surface(point_count, frame);

  const auto normalize_start = Clock::now();
  auto normalized = normalize_surface(input.view(), frame);
  const double normalize_ms = elapsed_ms(normalize_start);
  if (!normalized) {
    throw std::runtime_error("normalization benchmark failed");
  }

  const auto validity_start = Clock::now();
  auto validity = apply_validity_mask(normalized.value().view());
  const double validity_ms = elapsed_ms(validity_start);
  if (!validity) {
    throw std::runtime_error("validity benchmark failed");
  }

  const auto voxel_start = Clock::now();
  auto sampled = voxel_sample(validity.value().surface(), kVoxelSizeMm);
  const double voxel_sampling_ms = elapsed_ms(voxel_start);
  if (!sampled) {
    throw std::runtime_error("voxel benchmark failed");
  }

  const auto valid_points = validity.value().valid_count();
  const auto sampled_surface = sampled.value().surface();
  return BenchmarkResult{point_count,
                         valid_points,
                         sampled_surface.size(),
                         normalize_ms,
                         validity_ms,
                         voxel_sampling_ms,
                         peak_rss_bytes(),
                         result_checksum(sampled_surface, valid_points)};
}

[[nodiscard]] std::string compiler_name() {
#if defined(_MSC_VER)
  return "MSVC " + std::to_string(_MSC_VER);
#elif defined(__clang__)
  return "Clang " + std::string(__clang_version__);
#elif defined(__GNUC__)
  return "GCC " + std::string(__VERSION__);
#else
  return "unknown";
#endif
}

[[nodiscard]] std::string platform_name() {
#if defined(_WIN32)
  return "windows";
#elif defined(__linux__)
  return "linux";
#elif defined(__APPLE__)
  return "macos";
#else
  return "unknown";
#endif
}

[[nodiscard]] nlohmann::json to_json(const BenchmarkResult& result) {
  return nlohmann::json{{"input_points", result.input_points},
                        {"valid_points", result.valid_points},
                        {"sampled_points", result.sampled_points},
                        {"normalize_ms", result.normalize_ms},
                        {"validity_ms", result.validity_ms},
                        {"voxel_sampling_ms", result.voxel_sampling_ms},
                        {"total_ms", result.total_ms()},
                        {"peak_rss_bytes", result.peak_rss_bytes},
                        {"checksum", result.checksum}};
}

} // namespace

int main() {
  try {
    const auto frame = FrameId::create("benchmark").value();
    static_cast<void>(run_case(10'000U, frame));

    const auto small_result = run_case(kSmallPointCount, frame);
    const auto large_result = run_case(kLargePointCount, frame);
    const double scaling_ratio = large_result.total_ms() / std::max(small_result.total_ms(), 0.001);

    const bool within_limits = large_result.total_ms() <= kMaximumLargeTotalMs &&
                               scaling_ratio <= kMaximumScalingRatio &&
                               large_result.peak_rss_bytes <= kMaximumPeakRssBytes;
    const nlohmann::json document{
        {"schema_version", "1.0"},
        {"platform", platform_name()},
        {"compiler", compiler_name()},
        {"cases", nlohmann::json::array({to_json(small_result), to_json(large_result)})},
        {"scaling_ratio", scaling_ratio},
        {"limits",
         {{"large_total_ms", kMaximumLargeTotalMs},
          {"scaling_ratio", kMaximumScalingRatio},
          {"peak_rss_bytes", kMaximumPeakRssBytes}}},
        {"within_limits", within_limits}};
    std::cout << document.dump(2) << '\n';
    return within_limits ? 0 : 2;
  } catch (const std::exception& exception) {
    std::cerr << "benchmark failed: " << exception.what() << '\n';
    return 1;
  }
}
