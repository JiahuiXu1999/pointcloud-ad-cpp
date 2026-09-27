#pragma once

#include <cstdint>
#include <memory>
#include <pointcloud_ad/geometry.hpp>
#include <span>
#include <vector>

namespace pointcloud_ad::backends::cuda_backend {

[[nodiscard]] bool available() noexcept;

// Internal tiled strategy is retained only as a correctness/performance oracle.
enum class SearchStrategy : std::uint8_t { indexed, tiled };

// Private synchronous adapter. Owns a stream and per-solve device buffers; no CUDA types escape.
// Throws internally; the public registration entry point translates all failures to Result<T>.
class CorrespondenceSearch final {
public:
  CorrespondenceSearch(std::span<const Vec3f> reference, std::size_t query_capacity,
                       SearchStrategy strategy = SearchStrategy::indexed);
  ~CorrespondenceSearch();
  CorrespondenceSearch(const CorrespondenceSearch&) = delete;
  CorrespondenceSearch& operator=(const CorrespondenceSearch&) = delete;
  [[nodiscard]] std::vector<std::int32_t> query(std::span<const Vec3f> points,
                                                double max_distance_mm);
  void query_into(std::span<const Vec3f> points, double max_distance_mm,
                  std::vector<std::int32_t>& result);
  // Allocated device-array payload, excluding driver/context overhead and host construction.
  [[nodiscard]] std::size_t device_storage_bytes() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace pointcloud_ad::backends::cuda_backend
