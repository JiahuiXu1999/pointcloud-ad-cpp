#include "cpu_kernels.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

int main() {
  using namespace pointcloud_ad;
  using namespace pointcloud_ad::registration;
  using Clock = std::chrono::steady_clock;
  constexpr std::size_t count = 100000;
  std::vector<Vec3d> points(count), transformed(count);
  std::vector<ObjectiveContribution> contributions(count);
  for (std::size_t i = 0; i < count; ++i) {
    points[i] = {0.001 * static_cast<double>(i % 1000), 0.002 * static_cast<double>(i % 317),
                 0.003 * static_cast<double>(i % 113)};
  }
  const Matrix4 pose{1, 0, 0, 0.01, 0, 1, 0, -0.02, 0, 0, 1, 0.003, 0, 0, 0, 1};
  std::cout << "operation,kernel,points,p50_ms,checksum\n";
  for (int operation = 0; operation < 4; ++operation) {
    double expected = 0;
    for (auto requested : {CpuKernel::scalar, CpuKernel::avx2}) {
      if (requested == CpuKernel::avx2 && !avx2_available()) {
        continue;
      }
      const auto transform = transform_kernel(requested);
      const auto objective = objective_kernel(requested);
      std::vector<double> times;
      double checksum = 0;
      for (int repeat = -1; repeat < 7; ++repeat) {
        const auto begin = Clock::now();
        if (operation == 0) {
          transform(points, transformed, pose);
        } else {
          for (std::size_t i = 0; i < count; ++i) {
            ObjectiveInput input{
                points[i], {0, 0, 0},      {0, 0, 1},     {2, 0.2, 0.3, 0.2, 3, 0.4, 0.3, 0.4, 4},
                0.1,       operation == 1, operation == 3};
            objective(input, contributions[i]);
          }
        }
        const auto duration =
            std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
        checksum = 0;
        for (std::size_t i = 0; i < count; ++i) {
          if (operation == 0) {
            checksum += transformed[i].x + transformed[i].y + transformed[i].z;
          } else {
            for (double value : contributions[i].matrix) {
              checksum += value;
            }
            for (double value : contributions[i].gradient) {
              checksum += value;
            }
            checksum += contributions[i].absolute_residual;
          }
        }
        if (repeat >= 0) {
          times.push_back(duration);
        }
      }
      if (requested == CpuKernel::scalar) {
        expected = checksum;
      } else if (checksum != expected) {
        std::cerr << "kernel checksum differs\n";
        return 1;
      }
      std::sort(times.begin(), times.end());
      const char* names[] = {"transform", "point_to_plane", "point_to_point", "gicp"};
      std::cout << names[operation] << ',' << cpu_kernel_name(requested) << ',' << count << ','
                << times[times.size() / 2] << ',' << checksum << '\n';
    }
  }
}
