#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <pointcloud_ad/registration_engine.hpp>
#include <vector>

namespace {
using namespace pointcloud_ad;
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}
bool identical(const RegistrationMetrics& lhs, const RegistrationMetrics& rhs) {
  return lhs.final_transform().matrix() == rhs.final_transform().matrix() &&
         lhs.convergence() == rhs.convergence() && lhs.iterations() == rhs.iterations() &&
         lhs.valid_pairs() == rhs.valid_pairs() && lhs.fitness() == rhs.fitness() &&
         lhs.inlier_rmse_mm() == rhs.inlier_rmse_mm();
}
double median(std::array<double, 3> values) {
  std::sort(values.begin(), values.end());
  return values[1];
}
} // namespace
int main() {
  const auto reference_frame = FrameId::create("reference").value();
  const auto scan_frame = FrameId::create("scan").value();
  const auto initial = RigidTransform::create({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
                                              scan_frame, reference_frame)
                           .value();
  std::cout << "reference_points,scan_points,frames,method,backend,prepare_ms,first_frame_ms,later_"
               "frame_ms,context_total_ms,oneshot_total_ms\n";
  for (int side : {65, 129, 257}) {
    std::vector<Vec3f> points, normals;
    for (int row = 0; row < side; ++row) {
      for (int column = 0; column < side; ++column) {
        const double x = -4.0 + 8.0 * row / (side - 1);
        const double y = -4.0 + 8.0 * column / (side - 1);
        const double z = 0.08 * x * x + 0.13 * y * y;
        const double length = std::sqrt(0.16 * x * 0.16 * x + 0.26 * y * 0.26 * y + 1.0);
        points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
        normals.push_back({static_cast<float>(-0.16 * x / length),
                           static_cast<float>(-0.26 * y / length),
                           static_cast<float>(1.0 / length)});
      }
    }
    const auto reference =
        OwnedSurface::create(points, normals, {}, {}, LengthUnit::millimeter, reference_frame)
            .value();
    std::vector<OwnedSurface> scans;
    constexpr std::size_t frame_count = 5;
    for (std::size_t frame = 0; frame < frame_count; ++frame) {
      auto shifted = points;
      for (auto& point : shifted) {
        point.x -= 0.003F + 0.0001F * static_cast<float>(frame);
        point.y += 0.002F;
        point.z -= 0.01F + 0.0002F * static_cast<float>(frame);
      }
      scans.push_back(OwnedSurface::create(std::move(shifted), normals, {}, {},
                                           LengthUnit::millimeter, scan_frame)
                          .value());
    }
    for (auto method : {RegistrationMethod::point_to_plane, RegistrationMethod::point_to_point,
                        RegistrationMethod::gicp}) {
      for (auto backend : {ComputeBackend::cpu, ComputeBackend::gpu}) {
        if (!registration_backend_available(backend)) {
          continue;
        }
        RegistrationParameters options{40, 0.2, 0.1, 1.0e-7, 1.0e-8, 1.0e-9};
        options.method = method;
        std::array<double, 3> prepare{}, first{}, later{}, total{}, oneshot{};
        for (int run = -1; run < 3; ++run) {
          std::vector<RegistrationMetrics> expected;
          double oneshot_ms = 0;
          for (const auto& scan : scans) {
            const auto start = Clock::now();
            auto result =
                register_surfaces(reference.view(), scan.view(), initial, options, backend);
            oneshot_ms += elapsed(start);
            if (!result || result.value().convergence() != RegistrationConvergence::converged ||
                result.value().inlier_rmse_mm() > 0.001) {
              std::cerr << "one-shot correctness gate failed\n";
              return 1;
            }
            expected.push_back(std::move(result).value());
          }
          double prepare_ms = 0, first_ms = 0, later_ms = 0, cleanup_ms = 0;
          std::vector<RegistrationMetrics> actual;
          {
            const auto start = Clock::now();
            auto context =
                RegistrationContext::create(reference.view(), options, points.size(), backend);
            prepare_ms = elapsed(start);
            if (!context) {
              std::cerr << context.error().message << '\n';
              return 1;
            }
            for (std::size_t frame = 0; frame < frame_count; ++frame) {
              const auto begin = Clock::now();
              auto result = context.value().align(scans[frame].view(), initial);
              const double frame_ms = elapsed(begin);
              if (!result) {
                std::cerr << "context align failed\n";
                return 1;
              }
              if (frame == 0) {
                first_ms = frame_ms;
              } else {
                later_ms += frame_ms;
              }
              actual.push_back(std::move(result).value());
            }
            // Time destruction separately; correctness checks and result retention are excluded.
            const auto finish = Clock::now();
            {
              auto doomed = std::move(context).value();
            }
            cleanup_ms = elapsed(finish);
          }
          for (std::size_t frame = 0; frame < frame_count; ++frame) {
            if (!identical(actual[frame], expected[frame])) {
              std::cerr << "reused/fresh mismatch\n";
              return 1;
            }
          }
          if (run >= 0) {
            const auto sample = static_cast<std::size_t>(run);
            prepare[sample] = prepare_ms;
            first[sample] = first_ms;
            later[sample] = later_ms / static_cast<double>(frame_count - 1);
            total[sample] = prepare_ms + first_ms + later_ms + cleanup_ms;
            oneshot[sample] = oneshot_ms;
          }
        }
        std::cout << points.size() << ',' << points.size() << ',' << frame_count << ','
                  << registration_method_name(method) << ',' << compute_backend_name(backend) << ','
                  << median(prepare) << ',' << median(first) << ',' << median(later) << ','
                  << median(total) << ',' << median(oneshot) << std::endl;
      }
    }
  }
}
