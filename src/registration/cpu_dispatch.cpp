#include "cpu_dispatch.hpp"

#include <stdexcept>

#if defined(POINTCLOUDAD_CAN_BUILD_AVX2) && defined(_MSC_VER)
#include <intrin.h>
#elif defined(POINTCLOUDAD_CAN_BUILD_AVX2) && (defined(__x86_64__) || defined(__i386__))
#include <cpuid.h>
#endif

namespace pointcloud_ad::registration {
bool avx2_available() noexcept {
#if defined(POINTCLOUDAD_CAN_BUILD_AVX2) && defined(_MSC_VER)
  int registers[4]{};
  __cpuid(registers, 0);
  if (registers[0] < 7) {
    return false;
  }
  __cpuidex(registers, 1, 0);
  const auto ecx = static_cast<std::uint32_t>(registers[2]);
  if ((ecx & (1U << 27U)) == 0) {
    return false;
  }
  const auto xcr0 = _xgetbv(0);
  __cpuidex(registers, 7, 0);
  return avx2_state_supported(ecx, static_cast<std::uint32_t>(registers[1]), xcr0);
#elif defined(POINTCLOUDAD_CAN_BUILD_AVX2) && (defined(__x86_64__) || defined(__i386__))
  if (__get_cpuid_max(0, nullptr) < 7) {
    return false;
  }
  unsigned int eax{}, ebx{}, ecx{}, edx{};
  __cpuid_count(1, 0, eax, ebx, ecx, edx);
  const auto features = ecx;
  if ((features & (1U << 27U)) == 0) {
    return false;
  }
  unsigned int low{}, high{};
  __asm__ volatile("xgetbv" : "=a"(low), "=d"(high) : "c"(0));
  __cpuid_count(7, 0, eax, ebx, ecx, edx);
  return avx2_state_supported(features, ebx, (static_cast<std::uint64_t>(high) << 32U) | low);
#else
  return false;
#endif
}
CpuKernel select_cpu_kernel(CpuKernel request, bool available) {
  switch (request) {
  case CpuKernel::automatic:
    return available ? CpuKernel::avx2 : CpuKernel::scalar;
  case CpuKernel::scalar:
    return CpuKernel::scalar;
  case CpuKernel::avx2:
    if (available) {
      return CpuKernel::avx2;
    }
    throw std::invalid_argument("AVX2 requires a supported build, CPU and OS register state");
  }
  throw std::invalid_argument("unknown CPU kernel");
}
CpuKernel select_cpu_kernel(CpuKernel request) {
  return select_cpu_kernel(request, avx2_available());
}
} // namespace pointcloud_ad::registration
