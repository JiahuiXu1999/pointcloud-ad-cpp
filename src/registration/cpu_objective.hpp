#pragma once
#include "cpu_kernels.hpp"

#include <cmath>

namespace pointcloud_ad::registration::detail {
// Shared operation grouping is part of the scalar/SIMD equivalence contract.
// Operations provides only lane-wise products and six-column row arithmetic.
template <typename Operations>
void objective(const ObjectiveInput& input, ObjectiveContribution& output) noexcept {
  const Vec3d error{input.point.x - input.reference.x, input.point.y - input.reference.y,
                    input.point.z - input.reference.z};
  double products[3];
  Operations::products(error, input.plane ? input.normal : error, products);
  const double sum = (products[0] + products[1]) + products[2];
  const double residual = input.plane ? sum : std::sqrt(sum);
  output.absolute_residual = std::abs(residual);
  const double weight = output.absolute_residual <= input.huber_delta
                            ? 1.0
                            : input.huber_delta / output.absolute_residual;
  if (input.plane) {
    const auto p = input.point;
    const auto n = input.normal;
    const double jacobian[6]{
        p.y * n.z - p.z * n.y, p.z * n.x - p.x * n.z, p.x * n.y - p.y * n.x, n.x, n.y, n.z};
    for (std::size_t row = 0; row < 6; ++row) {
      Operations::outer_row(weight * jacobian[row], jacobian, output.matrix.data() + 6 * row);
      output.gradient[row] = (weight * (-residual)) * jacobian[row];
    }
    return;
  }
  const auto p = input.point;
  const double jacobian[18]{0, p.z, -p.y, 1, 0, 0, -p.z, 0, p.x, 0, 1, 0, p.y, -p.x, 0, 0, 0, 1};
  for (std::size_t row = 0; row < 6; ++row) {
    double weighted[3];
    for (std::size_t col = 0; col < 3; ++col) {
      if (!input.generalized) {
        weighted[col] = weight * jacobian[6 * col + row];
      } else {
        weighted[col] = (((weight * jacobian[row]) * input.information[col] +
                          (weight * jacobian[6 + row]) * input.information[3 + col]) +
                         (weight * jacobian[12 + row]) * input.information[6 + col]);
      }
    }
    Operations::matrix_row(weighted, jacobian, output.matrix.data() + 6 * row);
    output.gradient[row] =
        -((weighted[0] * error.x + weighted[1] * error.y) + weighted[2] * error.z);
  }
}
} // namespace pointcloud_ad::registration::detail
