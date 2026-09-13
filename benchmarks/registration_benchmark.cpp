#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <pointcloud_ad/registration_engine.hpp>
#include <vector>

int main() {
  using namespace pointcloud_ad;
  const auto reference_frame = FrameId::create("reference").value();
  const auto scan_frame = FrameId::create("scan").value();
  const auto initial = RigidTransform::create({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
                                              scan_frame, reference_frame)
                           .value();
  std::cout << "points,method,backend,median_ms,iterations,rmse_mm\n";
  for (int side : {17, 33, 65}) {
    std::vector<Vec3f> points, scan_points, normals;
    for (int row = 0; row < side; ++row) {
      for (int column = 0; column < side; ++column) {
        const double x = -4.0 + 8.0 * row / (side - 1);
        const double y = -4.0 + 8.0 * column / (side - 1);
        const double z = 0.08 * x * x + 0.13 * y * y;
        points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
        scan_points.push_back({static_cast<float>(x - 0.03), static_cast<float>(y + 0.02),
                               static_cast<float>(z - 0.04)});
        const double length = std::sqrt(0.16 * x * 0.16 * x + 0.26 * y * 0.26 * y + 1.0);
        normals.push_back({static_cast<float>(-0.16 * x / length),
                           static_cast<float>(-0.26 * y / length),
                           static_cast<float>(1.0 / length)});
      }
    }
    const auto reference = OwnedSurface::create(points, normals, {}, std::nullopt,
                                                LengthUnit::millimeter, reference_frame)
                               .value();
    const auto scan = OwnedSurface::create(scan_points, normals, {}, std::nullopt,
                                           LengthUnit::millimeter, scan_frame)
                          .value();
    for (auto method : {RegistrationMethod::point_to_plane, RegistrationMethod::point_to_point,
                        RegistrationMethod::gicp}) {
      for (auto backend : {ComputeBackend::cpu, ComputeBackend::gpu}) {
        if (!registration_backend_available(backend)) {
          continue;
        }
        RegistrationParameters parameters{40, 1.0, 0.5, 1.0e-7, 1.0e-8, 1.0e-9};
        parameters.method = method;
        std::array<double, 3> times{};
        std::uint32_t iterations = 0;
        double rmse = 0;
        // One warm-up excludes driver initialization; every measured solve still includes
        // per-solve preparation, allocation, transfers, final evaluation and cleanup.
        for (int run = -1; run < 3; ++run) {
          const auto start = std::chrono::steady_clock::now();
          auto result =
              register_surfaces(reference.view(), scan.view(), initial, parameters, backend);
          const double ms =
              std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                  .count();
          if (!result || result.value().convergence() != RegistrationConvergence::converged ||
              result.value().inlier_rmse_mm() > 0.001) {
            std::cerr << "registration benchmark correctness gate failed\n";
            return 1;
          }
          if (run >= 0) {
            times[static_cast<std::size_t>(run)] = ms;
          }
          iterations = result.value().iterations();
          rmse = result.value().inlier_rmse_mm();
        }
        std::sort(times.begin(), times.end());
        std::cout << points.size() << ',' << registration_method_name(method) << ','
                  << compute_backend_name(backend) << ',' << times[1] << ',' << iterations << ','
                  << rmse << '\n';
      }
    }
  }
}
