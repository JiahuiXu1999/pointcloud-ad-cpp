#pragma once
#include <cstdint>
#include <pointcloud_ad/registration_options.hpp>

namespace pointcloud_ad::registration {
constexpr bool avx2_state_supported(std::uint32_t leaf1_ecx, std::uint32_t leaf7_ebx,
                                    std::uint64_t xcr0) noexcept {
  constexpr std::uint32_t required = (1U << 26U) | (1U << 27U) | (1U << 28U);
  return (leaf1_ecx & required) == required && (leaf7_ebx & (1U << 5U)) != 0 && (xcr0 & 6U) == 6U;
}
bool avx2_available() noexcept;
CpuKernel select_cpu_kernel(CpuKernel request, bool available);
CpuKernel select_cpu_kernel(CpuKernel request);
} // namespace pointcloud_ad::registration
