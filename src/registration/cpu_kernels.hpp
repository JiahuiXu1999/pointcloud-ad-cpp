#pragma once
#include "cpu_dispatch.hpp"

#include <array>
#include <pointcloud_ad/geometry.hpp>
#include <span>

namespace pointcloud_ad::registration {
using Matrix4 = std::array<double, 16>;
// Private kernels require equal input/output sizes; exact in-place transform is allowed.
using TransformKernel = void (*)(std::span<const Vec3d>, std::span<Vec3d>, const Matrix4&) noexcept;
void transform_scalar(std::span<const Vec3d> input, std::span<Vec3d> output,
                      const Matrix4& matrix) noexcept;
void transform_avx2(std::span<const Vec3d> input, std::span<Vec3d> output,
                    const Matrix4& matrix) noexcept;
TransformKernel transform_kernel(CpuKernel resolved) noexcept;

struct ObjectiveInput {
  Vec3d point, reference, normal;
  std::array<double, 9> information{1, 0, 0, 0, 1, 0, 0, 0, 1}; // Row-major; GICP only.
  double huber_delta;
  bool plane;
  bool generalized{true};
};
struct ObjectiveContribution {
  std::array<double, 36> matrix; // Row-major, completely assigned by the kernel.
  std::array<double, 6> gradient;
  double absolute_residual;
};
using ObjectiveKernel = void (*)(const ObjectiveInput&, ObjectiveContribution&) noexcept;
void objective_scalar(const ObjectiveInput& input, ObjectiveContribution& output) noexcept;
void objective_avx2(const ObjectiveInput& input, ObjectiveContribution& output) noexcept;
ObjectiveKernel objective_kernel(CpuKernel resolved) noexcept;
} // namespace pointcloud_ad::registration
