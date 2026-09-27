#include "cpu_dispatch.hpp"

#include <iostream>
#include <stdexcept>

int main() {
  using namespace pointcloud_ad;
  using namespace pointcloud_ad::registration;
  constexpr std::uint32_t flags = (1U << 26U) | (1U << 27U) | (1U << 28U);
  bool passed = avx2_state_supported(flags, 1U << 5U, 6);
  for (auto bit : {26U, 27U, 28U}) {
    passed &= !avx2_state_supported(flags & ~(1U << bit), 1U << 5U, 6);
  }
  passed &= !avx2_state_supported(flags, 0, 6) && !avx2_state_supported(flags, 32, 2) &&
            !avx2_state_supported(flags, 32, 4);
  passed &= select_cpu_kernel(CpuKernel::automatic, false) == CpuKernel::scalar;
  passed &= select_cpu_kernel(CpuKernel::automatic, true) == CpuKernel::avx2;
  passed &= select_cpu_kernel(CpuKernel::scalar, true) == CpuKernel::scalar;
  bool rejected = false;
  try {
    (void)select_cpu_kernel(CpuKernel::avx2, false);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  passed &= rejected;
  std::cout << "AVX2 available: " << avx2_available() << '\n';
  return passed ? 0 : 1;
}
