#include "cpu_kernels.hpp"
#include "cpu_objective.hpp"

#include <immintrin.h>

namespace pointcloud_ad::registration {
void transform_avx2(std::span<const Vec3d> input, std::span<Vec3d> output,
                    const Matrix4& m) noexcept {
  std::size_t i = 0;
  for (; i + 4 <= input.size(); i += 4) {
    const auto x = _mm256_set_pd(input[i + 3].x, input[i + 2].x, input[i + 1].x, input[i].x);
    const auto y = _mm256_set_pd(input[i + 3].y, input[i + 2].y, input[i + 1].y, input[i].y);
    const auto z = _mm256_set_pd(input[i + 3].z, input[i + 2].z, input[i + 1].z, input[i].z);
    alignas(32) double values[3][4];
    for (std::size_t row = 0; row < 3; ++row) {
      auto result = _mm256_add_pd(_mm256_mul_pd(_mm256_set1_pd(m[4 * row]), x),
                                  _mm256_mul_pd(_mm256_set1_pd(m[4 * row + 1]), y));
      result = _mm256_add_pd(result, _mm256_mul_pd(_mm256_set1_pd(m[4 * row + 2]), z));
      result = _mm256_add_pd(result, _mm256_set1_pd(m[4 * row + 3]));
      _mm256_store_pd(values[row], result);
    }
    for (std::size_t lane = 0; lane < 4; ++lane) {
      output[i + lane] = {values[0][lane], values[1][lane], values[2][lane]};
    }
  }
  transform_scalar(input.subspan(i), output.subspan(i), m);
}

namespace {
struct AvxOperations {
  static void products(Vec3d left, Vec3d right, double* result) noexcept {
    const auto a = _mm256_set_pd(0, left.z, left.y, left.x);
    const auto b = _mm256_set_pd(0, right.z, right.y, right.x);
    alignas(32) double values[4];
    _mm256_store_pd(values, _mm256_mul_pd(a, b));
    for (std::size_t i = 0; i < 3; ++i) {
      result[i] = values[i];
    }
  }
  static void outer_row(double weight, const double* jacobian, double* result) noexcept {
    _mm256_storeu_pd(result, _mm256_mul_pd(_mm256_set1_pd(weight), _mm256_loadu_pd(jacobian)));
    _mm_storeu_pd(result + 4, _mm_mul_pd(_mm_set1_pd(weight), _mm_loadu_pd(jacobian + 4)));
  }
  static void matrix_row(const double* weight, const double* jacobian, double* result) noexcept {
    auto first =
        _mm256_add_pd(_mm256_mul_pd(_mm256_set1_pd(weight[0]), _mm256_loadu_pd(jacobian)),
                      _mm256_mul_pd(_mm256_set1_pd(weight[1]), _mm256_loadu_pd(jacobian + 6)));
    first = _mm256_add_pd(first,
                          _mm256_mul_pd(_mm256_set1_pd(weight[2]), _mm256_loadu_pd(jacobian + 12)));
    _mm256_storeu_pd(result, first);
    auto tail = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(weight[0]), _mm_loadu_pd(jacobian + 4)),
                           _mm_mul_pd(_mm_set1_pd(weight[1]), _mm_loadu_pd(jacobian + 10)));
    tail = _mm_add_pd(tail, _mm_mul_pd(_mm_set1_pd(weight[2]), _mm_loadu_pd(jacobian + 16)));
    _mm_storeu_pd(result + 4, tail);
  }
};
} // namespace
void objective_avx2(const ObjectiveInput& input, ObjectiveContribution& output) noexcept {
  detail::objective<AvxOperations>(input, output);
}
} // namespace pointcloud_ad::registration
