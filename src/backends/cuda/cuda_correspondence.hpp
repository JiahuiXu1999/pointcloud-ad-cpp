#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <pointcloud_ad/geometry.hpp>
#include <span>
#include <vector>

namespace pointcloud_ad::backends::cuda_backend {

[[nodiscard]] bool available() noexcept;

// Internal tiled strategy is retained only as a correctness/performance oracle.
enum class SearchStrategy : std::uint8_t { indexed, tiled };

// Row-major 6x6 equation, six-vector RHS, residual sum, count and invalid-coordinate count.
// Trivially copyable device/host transfer packet; no public ABI.
struct TransferStatistics final {
  std::uint64_t host_to_device_bytes{};
  std::uint64_t device_to_host_bytes{};
  std::uint64_t equation_evaluations{};
};

struct Covariance final {
  double values[9]{};
};

struct Equation final {
  double values[45]{};
};

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
  // Opt-in resident arithmetic. Normals are normalized doubles in original reference order.
  // Empty normals disable back-face rejection. One scan upload per align; pose changes per
  // iteration.
  void prepare_resident(std::span<const Vec3d> reference_normals);
  // Exact self-KNN ordered by (double squared distance, original index). Includes self.
  [[nodiscard]] std::vector<std::int32_t> knn(std::uint32_t neighbors);
  void prepare_gicp(std::uint32_t neighbors, double epsilon);
  [[nodiscard]] std::vector<Covariance> download_covariances();
  void upload_scan(std::span<const Vec3d> points, std::span<const Vec3d> normals);
  [[nodiscard]] Equation evaluate(const std::array<double, 16>& pose, double max_distance_mm,
                                  double huber_delta_mm, bool plane, bool final_metrics);
  // Allocated device-array payload, excluding driver/context overhead and host construction.
  [[nodiscard]] std::size_t device_storage_bytes() const noexcept;
  [[nodiscard]] TransferStatistics transfer_statistics() const noexcept;

private:
  void prepare_knn(std::uint32_t neighbors);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace pointcloud_ad::backends::cuda_backend
