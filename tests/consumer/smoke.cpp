#include <iostream>
#include <optional>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/normalization.hpp>
#include <pointcloud_ad/registration_engine.hpp>
#include <pointcloud_ad/result.hpp>
#include <pointcloud_ad/status.hpp>
#include <pointcloud_ad/surface.hpp>
#include <pointcloud_ad/version.hpp>
#include <utility>

int main() {
  if (!pointcloud_ad::registration_backend_available(pointcloud_ad::ComputeBackend::cpu)) {
    return 1;
  }

  auto frame = pointcloud_ad::FrameId::create("consumer");
  if (!frame) {
    return 1;
  }
  auto surface = pointcloud_ad::OwnedSurface::create({pointcloud_ad::Vec3f{}}, {}, {}, std::nullopt,
                                                     pointcloud_ad::LengthUnit::millimeter,
                                                     std::move(frame).value());
  if (!surface || surface.value().size() != 1U) {
    return 1;
  }
  pointcloud_ad::RegistrationParameters options{10, 1.0, 0.5, 1.0e-6, 1.0e-6, 1.0e-6};
  options.method = pointcloud_ad::RegistrationMethod::point_to_point;
  options.thread_count = 2;
  options.cpu_kernel = pointcloud_ad::CpuKernel::scalar;
  auto initial = pointcloud_ad::RigidTransform::create(
      {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, surface.value().view().frame(),
      surface.value().view().frame());
  if (!initial) {
    return 1;
  }
  for (auto backend : {pointcloud_ad::ComputeBackend::cpu, pointcloud_ad::ComputeBackend::gpu}) {
    if (!pointcloud_ad::registration_backend_available(backend)) {
      continue;
    }
    auto context =
        pointcloud_ad::RegistrationContext::create(surface.value().view(), options, 1, backend);
    if (!context) {
      return 1;
    }
    auto moved = std::move(context).value();
    context.value() = std::move(moved);
    auto registered = context.value().align(surface.value().view(), initial.value());
    if (!registered || registered.value().convergence() !=
                           pointcloud_ad::RegistrationConvergence::degenerate_input) {
      return 1;
    }
  }
  std::cout << pointcloud_ad::version_string() << '\n';
  return pointcloud_ad::version().major == 0 ? 0 : 1;
}
