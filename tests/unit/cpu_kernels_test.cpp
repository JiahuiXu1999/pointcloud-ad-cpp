#include "cpu_kernels.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace {
bool check_objective(pointcloud_ad::registration::ObjectiveInput input) {
  using namespace pointcloud_ad;
  using namespace pointcloud_ad::registration;
  ObjectiveContribution scalar{}, vectorized{};
  objective_scalar(input, scalar);
  objective_kernel(select_cpu_kernel(CpuKernel::automatic))(input, vectorized);
  bool passed = scalar.matrix == vectorized.matrix && scalar.gradient == vectorized.gradient &&
                scalar.absolute_residual == vectorized.absolute_residual;
  const std::array<double, 3> error{input.point.x - input.reference.x,
                                    input.point.y - input.reference.y,
                                    input.point.z - input.reference.z};
  const std::array<double, 3> normal{input.normal.x, input.normal.y, input.normal.z};
  const double residual =
      input.plane ? error[0] * normal[0] + error[1] * normal[1] + error[2] * normal[2]
                  : std::sqrt(error[0] * error[0] + error[1] * error[1] + error[2] * error[2]);
  const double weight =
      std::abs(residual) <= input.huber_delta ? 1 : input.huber_delta / std::abs(residual);
  std::array<std::array<double, 3>, 6> derivative{};
  // Independent central differences of the infinitesimal rigid-motion residual.
  const auto evaluate = [&](std::size_t axis, double delta) {
    std::array<double, 3> rotation{}, translation{};
    if (axis < 3) {
      rotation[axis] = delta;
    } else {
      translation[axis - 3] = delta;
    }
    return std::array<double, 3>{
        input.point.x + rotation[1] * input.point.z - rotation[2] * input.point.y + translation[0] -
            input.reference.x,
        input.point.y + rotation[2] * input.point.x - rotation[0] * input.point.z + translation[1] -
            input.reference.y,
        input.point.z + rotation[0] * input.point.y - rotation[1] * input.point.x + translation[2] -
            input.reference.z};
  };
  constexpr double step = 1e-5;
  for (std::size_t axis = 0; axis < 6; ++axis) {
    const auto plus = evaluate(axis, step), minus = evaluate(axis, -step);
    for (std::size_t k = 0; k < 3; ++k) {
      derivative[axis][k] = (plus[k] - minus[k]) / (2 * step);
    }
  }
  for (std::size_t row = 0; row < 6; ++row) {
    double gradient = 0;
    for (std::size_t a = 0; a < 3; ++a) {
      for (std::size_t b = 0; b < 3; ++b) {
        const double info =
            input.plane ? normal[a] * normal[b]
                        : (input.generalized ? input.information[a * 3 + b] : (a == b ? 1.0 : 0.0));
        gradient -= weight * derivative[row][a] * info * error[b];
      }
    }
    passed &= std::abs(gradient - scalar.gradient[row]) < 1e-8;
    for (std::size_t col = 0; col < 6; ++col) {
      double hessian = 0;
      for (std::size_t a = 0; a < 3; ++a) {
        for (std::size_t b = 0; b < 3; ++b) {
          const double info = input.plane ? normal[a] * normal[b]
                                          : (input.generalized ? input.information[a * 3 + b]
                                                               : (a == b ? 1.0 : 0.0));
          hessian += weight * derivative[row][a] * info * derivative[col][b];
        }
      }
      passed &= std::abs(hessian - scalar.matrix[row * 6 + col]) < 1e-8;
    }
  }
  return passed;
}
} // namespace

int main() {
  using namespace pointcloud_ad;
  using namespace pointcloud_ad::registration;
  const Matrix4 matrix{0.6, -0.8, 0, 0.003, 0.8, 0.6, 0, -2.3, 0, 0, 1, 0.17, 0, 0, 0, 1};
  const auto kernel = transform_kernel(select_cpu_kernel(CpuKernel::automatic));
  bool passed = true;
  for (std::size_t count : {0U, 1U, 2U, 3U, 4U, 5U, 7U, 255U, 256U, 257U, 1089U}) {
    // Offset by one Vec3d; callers need not supply 32-byte alignment.
    std::vector<Vec3d> storage(count + 2, Vec3d{91, 92, 93});
    auto input = std::span<Vec3d>(storage).subspan(1, count);
    for (std::size_t i = 0; i < count; ++i) {
      input[i] = {std::sin(static_cast<double>(i)), std::cos(static_cast<double>(i)),
                  static_cast<double>(i) * 0.123};
    }
    std::vector<Vec3d> expected(count);
    transform_scalar(input, expected, matrix);
    kernel(input, input, matrix);
    for (std::size_t i = 0; i < count; ++i) {
      passed &=
          input[i].x == expected[i].x && input[i].y == expected[i].y && input[i].z == expected[i].z;
    }
    passed &= storage.front().x == 91 && storage.back().z == 93;
  }
  for (bool plane : {false, true}) {
    for (double residual :
         {0.0, -0.1, std::nextafter(0.1, 0.0), 0.1, std::nextafter(0.1, 1.0), 0.5}) {
      ObjectiveInput input{{0.3, -0.4, residual},
                           {0.3, -0.4, 0.0},
                           {0, 0, 1},
                           {1, 0, 0, 0, 1, 0, 0, 0, 1},
                           0.1,
                           plane};
      passed &= check_objective(input);
      input.generalized = false;
      passed &= check_objective(input);
      input.generalized = true;
      input.normal = {0.6, 0, 0.8};
      input.information = {2, 0.2, 0.3, 0.2, 3, 0.4, 0.3, 0.4, 4};
      passed &= check_objective(input);
    }
  }
  if (!passed) {
    std::cerr << "SIMD transform parity/tail/guard failed\n";
  }
  return passed ? 0 : 1;
}
