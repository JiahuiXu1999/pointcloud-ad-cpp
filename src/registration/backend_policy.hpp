#pragma once

#include <cstddef>
#include <pointcloud_ad/registration_options.hpp>

namespace pointcloud_ad::registration {
// Conservative warm-registration threshold from M14/M15 measurements, not a universal
// latency guarantee. Context capacity is fixed; later frames do not switch backend.
inline constexpr std::size_t automatic_gpu_min_points = 65536;

[[nodiscard]] constexpr bool gpu_size_eligible(std::size_t reference_points,
                                               std::size_t scan_capacity) noexcept {
  return reference_points >= automatic_gpu_min_points && scan_capacity >= automatic_gpu_min_points;
}

[[nodiscard]] constexpr ComputeBackend select_backend(ComputeBackend requested,
                                                      std::size_t reference_points,
                                                      std::size_t scan_capacity,
                                                      bool gpu_available) noexcept {
  if (requested != ComputeBackend::automatic)
    return requested;
  return gpu_available && gpu_size_eligible(reference_points, scan_capacity) ? ComputeBackend::gpu
                                                                             : ComputeBackend::cpu;
}
} // namespace pointcloud_ad::registration
