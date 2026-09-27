#include "cpu_kernels.hpp"

#include "cpu_objective.hpp"

namespace pointcloud_ad::registration {
void transform_scalar(std::span<const Vec3d> input, std::span<Vec3d> output,
                      const Matrix4& m) noexcept {
  for (std::size_t i = 0; i < input.size(); ++i) {
    const auto p = input[i];
    output[i] = {((m[0] * p.x + m[1] * p.y) + m[2] * p.z) + m[3],
                 ((m[4] * p.x + m[5] * p.y) + m[6] * p.z) + m[7],
                 ((m[8] * p.x + m[9] * p.y) + m[10] * p.z) + m[11]};
  }
}
TransformKernel transform_kernel(CpuKernel resolved) noexcept {
#ifdef POINTCLOUDAD_CAN_BUILD_AVX2
  if (resolved == CpuKernel::avx2) {
    return transform_avx2;
  }
#else
  (void)resolved;
#endif
  return transform_scalar;
}

namespace {
struct ScalarOperations {
  static void products(Vec3d left, Vec3d right, double* result) noexcept {
    result[0] = left.x * right.x;
    result[1] = left.y * right.y;
    result[2] = left.z * right.z;
  }
  static void outer_row(double weight, const double* jacobian, double* result) noexcept {
    for (std::size_t col = 0; col < 6; ++col) {
      result[col] = weight * jacobian[col];
    }
  }
  static void matrix_row(const double* weight, const double* jacobian, double* result) noexcept {
    for (std::size_t col = 0; col < 6; ++col) {
      result[col] = (weight[0] * jacobian[col] + weight[1] * jacobian[6 + col]) +
                    weight[2] * jacobian[12 + col];
    }
  }
};
} // namespace
void objective_scalar(const ObjectiveInput& input, ObjectiveContribution& output) noexcept {
  detail::objective<ScalarOperations>(input, output);
}
ObjectiveKernel objective_kernel(CpuKernel resolved) noexcept {
#ifdef POINTCLOUDAD_CAN_BUILD_AVX2
  if (resolved == CpuKernel::avx2) {
    return objective_avx2;
  }
#else
  (void)resolved;
#endif
  return objective_scalar;
}
} // namespace pointcloud_ad::registration
