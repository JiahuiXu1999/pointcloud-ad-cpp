#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <pointcloud_ad/registration_engine.hpp>
#include <string_view>
#include <utility>
#include <vector>

namespace {
using namespace pointcloud_ad;
constexpr std::array<double, 16> kIdentity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

OwnedSurface patch(const FrameId& frame, bool scan, bool normals) {
  std::vector<Vec3f> points;
  std::vector<Vec3f> directions;
  const double angle = 0.015;
  const double c = std::cos(angle), s = std::sin(angle);
  for (int x = -8; x <= 8; ++x) {
    for (int y = -8; y <= 8; ++y) {
      const double px = 0.5 * x, py = 0.5 * y;
      const double pz = 0.08 * px * px + 0.13 * py * py + 0.003 * px * py * py;
      double nx = -0.16 * px - 0.003 * py * py;
      double ny = -0.26 * py - 0.006 * px * py;
      const double length = std::sqrt(nx * nx + ny * ny + 1.0);
      nx /= length;
      ny /= length;
      if (scan) {
        points.push_back({static_cast<float>(c * (px - 0.12) + s * (py + 0.08)),
                          static_cast<float>(-s * (px - 0.12) + c * (py + 0.08)),
                          static_cast<float>(pz - 0.10)});
        if (normals) {
          directions.push_back({static_cast<float>(c * nx + s * ny),
                                static_cast<float>(-s * nx + c * ny),
                                static_cast<float>(1.0 / length)});
        }
      } else {
        points.push_back({static_cast<float>(px), static_cast<float>(py), static_cast<float>(pz)});
        if (normals) {
          directions.push_back(
              {static_cast<float>(nx), static_cast<float>(ny), static_cast<float>(1.0 / length)});
        }
      }
    }
  }
  std::vector<std::uint8_t> mask(points.size(), 1U);
  points.push_back({std::numeric_limits<float>::quiet_NaN(), 0, 0});
  mask.push_back(0U);
  if (normals) {
    directions.push_back({0, 0, 0});
  }
  return OwnedSurface::create(std::move(points), std::move(directions), std::move(mask),
                              std::nullopt, LengthUnit::millimeter, frame)
      .value();
}

RegistrationParameters parameters(RegistrationMethod method) {
  RegistrationParameters p;
  p.max_iterations = 100;
  p.max_correspondence_distance_mm = 1.0;
  p.huber_delta_mm = 0.5;
  p.translation_epsilon_mm = 1.0e-8;
  p.rotation_epsilon_rad = 1.0e-9;
  p.residual_epsilon_mm = 1.0e-10;
  p.method = method;
  return p;
}
} // namespace

int main() {
  using namespace pointcloud_ad;
  bool passed = true;
  const auto reference_frame = FrameId::create("reference").value();
  const auto scan_frame = FrameId::create("scan").value();
  const auto initial = RigidTransform::create(kIdentity, scan_frame, reference_frame).value();
  const auto reference = patch(reference_frame, false, true);
  const auto scan = patch(scan_frame, true, true);
  const auto bare_reference = patch(reference_frame, false, false);
  const auto bare_scan = patch(scan_frame, true, false);
  const bool gpu_available = registration_backend_available(ComputeBackend::gpu);
#ifdef POINTCLOUDAD_TEST_CUDA
  passed &= expect(gpu_available, "CUDA build acceptance requires an actual accessible GPU");
#endif
  passed &= expect(!registration_backend_available(static_cast<ComputeBackend>(255)),
                   "invalid backend rejected");
  const std::array methods{RegistrationMethod::point_to_plane, RegistrationMethod::point_to_point,
                           RegistrationMethod::gicp};
  for (const auto method : methods) {
    auto p = parameters(method);
    auto cpu = register_surfaces(reference.view(), scan.view(), initial, p, ComputeBackend::cpu);
    if (!cpu) {
      std::cerr << registration_method_name(method) << ": " << cpu.error().message << '\n';
      for (const auto& [key, value] : cpu.error().context) {
        std::cerr << key << ": " << value << '\n';
      }
      passed = false;
      continue;
    }
    const auto& pose = cpu.value().final_transform().matrix();
    passed &=
        expect(cpu.value().convergence() == RegistrationConvergence::converged, "CPU convergence");
    passed &=
        expect(std::abs(pose[3] - 0.12) < 0.002 && std::abs(pose[7] + 0.08) < 0.002 &&
                   std::abs(pose[11] - 0.10) < 0.002 && std::abs(pose[1] + std::sin(0.015)) < 0.002,
               "known nonidentity scan-to-reference pose recovered");
    passed &=
        expect(cpu.value().valid_pairs() == reference.view().size() - 1U, "masked NaN excluded");
    auto repeat =
        register_surfaces(reference.view(), scan.view(), initial, p, ComputeBackend::automatic);
    passed &= expect(repeat && repeat.value().final_transform().matrix() == pose,
                     "automatic selects CPU and repeats exactly");
    if (method != RegistrationMethod::point_to_plane) {
      auto bare = register_surfaces(bare_reference.view(), bare_scan.view(), initial, p);
      passed &= expect(bare && bare.value().inlier_rmse_mm() < 0.002,
                       "point-to-point and GICP accept points without normals");
    }
    auto gpu = register_surfaces(reference.view(), scan.view(), initial, p, ComputeBackend::gpu);
    if (gpu_available) {
      passed &= expect(static_cast<bool>(gpu), "GPU algorithm executes");
      if (gpu) {
        for (std::size_t i = 0; i < pose.size(); ++i) {
          passed &= expect(std::abs(pose[i] - gpu.value().final_transform().matrix()[i]) < 1.0e-5,
                           "CPU/GPU pose tolerance");
        }
        passed &= expect(cpu.value().valid_pairs() == gpu.value().valid_pairs() &&
                             cpu.value().convergence() == gpu.value().convergence() &&
                             std::abs(cpu.value().inlier_rmse_mm() - gpu.value().inlier_rmse_mm()) <
                                 1.0e-5,
                         "CPU/GPU metrics agree");
        auto repeated_gpu =
            register_surfaces(reference.view(), scan.view(), initial, p, ComputeBackend::gpu);
        passed &= expect(repeated_gpu && repeated_gpu.value().final_transform().matrix() ==
                                             gpu.value().final_transform().matrix(),
                         "GPU repeated pose identical");
      }
    } else {
      passed &= expect(!gpu && gpu.error().code == ErrorCode::invalid_argument,
                       "explicit unavailable GPU fails without fallback");
    }
    p.max_correspondence_distance_mm = 1.0e-6;
    auto no_pairs = register_surfaces(reference.view(), scan.view(), initial, p);
    passed &=
        expect(no_pairs && no_pairs.value().valid_pairs() == 0U &&
                   no_pairs.value().convergence() == RegistrationConvergence::degenerate_input,
               "no correspondence never converges");
  }
  auto p = parameters(RegistrationMethod::point_to_plane);
  passed &= expect(!register_surfaces(bare_reference.view(), bare_scan.view(), initial, p),
                   "point-to-plane rejects missing normals");
  p.method = static_cast<RegistrationMethod>(255);
  passed &= expect(!register_surfaces(reference.view(), scan.view(), initial, p),
                   "invalid method rejected");
  p = parameters(RegistrationMethod::gicp);
  p.covariance_epsilon = 0.0;
  passed &= expect(!register_surfaces(reference.view(), scan.view(), initial, p),
                   "invalid covariance rejected");
  p = parameters(RegistrationMethod::point_to_point);
  const auto wrong = RigidTransform::create(kIdentity, reference_frame, scan_frame).value();
  passed &= expect(!register_surfaces(reference.view(), scan.view(), wrong, p),
                   "wrong frame direction rejected");
  p.max_iterations = 0;
  passed &= expect(!register_surfaces(reference.view(), scan.view(), initial, p),
                   "zero iteration rejected");
  {
    std::vector<Vec3f> line;
    for (int index = 0; index < 20; ++index) {
      line.push_back({static_cast<float>(index), 0, 0});
    }
    const auto line_reference =
        OwnedSurface::create(line, {}, {}, std::nullopt, LengthUnit::millimeter, reference_frame)
            .value();
    const auto line_scan =
        OwnedSurface::create(line, {}, {}, std::nullopt, LengthUnit::millimeter, scan_frame)
            .value();
    auto point_result = register_surfaces(line_reference.view(), line_scan.view(), initial,
                                          parameters(RegistrationMethod::point_to_point));
    passed &= expect(point_result && point_result.value().convergence() ==
                                         RegistrationConvergence::degenerate_input,
                     "collinear point-to-point geometry is not accepted as converged");
    passed &= expect(!register_surfaces(line_reference.view(), line_scan.view(), initial,
                                        parameters(RegistrationMethod::gicp)),
                     "collinear GICP covariance rejected");
    const auto meters =
        OwnedSurface::create(line, {}, {}, std::nullopt, LengthUnit::meter, scan_frame).value();
    passed &= expect(!register_surfaces(line_reference.view(), meters.view(), initial,
                                        parameters(RegistrationMethod::point_to_point)),
                     "unnormalized units rejected");
  }
  return passed ? 0 : 1;
}
