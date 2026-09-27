#pragma once

#include <cstdint>
#include <string_view>

namespace pointcloud_ad {

enum class RegistrationMethod : std::uint8_t { point_to_plane, point_to_point, gicp };
// CPU arithmetic selection, independent of CPU/CUDA correspondence search.
// automatic selects a supported implementation; forced unavailable AVX2 fails.
enum class CpuKernel : std::uint8_t { automatic, scalar, avx2 };

[[nodiscard]] constexpr std::string_view cpu_kernel_name(CpuKernel kernel) noexcept {
  switch (kernel) {
  case CpuKernel::automatic:
    return "auto";
  case CpuKernel::scalar:
    return "scalar";
  case CpuKernel::avx2:
    return "avx2";
  }
  return "unknown";
}

enum class ComputeBackend : std::uint8_t { cpu, gpu, automatic };

[[nodiscard]] constexpr std::string_view
registration_method_name(RegistrationMethod method) noexcept {
  switch (method) {
  case RegistrationMethod::point_to_plane:
    return "point_to_plane";
  case RegistrationMethod::point_to_point:
    return "point_to_point";
  case RegistrationMethod::gicp:
    return "gicp";
  }
  return "unknown";
}

[[nodiscard]] constexpr std::string_view compute_backend_name(ComputeBackend backend) noexcept {
  switch (backend) {
  case ComputeBackend::cpu:
    return "cpu";
  case ComputeBackend::gpu:
    return "gpu";
  case ComputeBackend::automatic:
    return "auto";
  }
  return "unknown";
}

} // namespace pointcloud_ad
