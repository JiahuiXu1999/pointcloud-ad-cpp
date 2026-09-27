#include "cpu_dispatch.hpp"
#include "performance_fixture.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
  using namespace pointcloud_ad;
  const auto rf = FrameId::create("reference").value();
  const auto sf = FrameId::create("scan").value();
  const auto reference = performance::patch(rf, 33, false, true);
  const auto scan = performance::patch(sf, 33, true, true);
  const auto initial = performance::initial(sf, rf);
  bool passed = true;
  std::size_t method_index = 0;
  constexpr std::array<std::array<double, 16>, 3> baseline{
      {{{0.99999999999998568, 1.4551115378572414e-14, -1.6920887011115639e-07, 0.003010600629848429,
         -9.024642577792918e-15, 0.99999999999999944, 3.2660656602394294e-08,
         -0.0020013210741826057, 1.6920887011115676e-07, -3.2660656602392296e-08,
         0.99999999999998535, 0.0099999987936659485, 0, 0, 0, 1}},
       {{1, -2.9514334392654649e-20, -8.2606465157050444e-11, 0.0030000108722892365,
         2.9451162271958817e-23, 1, -3.5693190810581679e-10, -0.0020000318903881679,
         8.2606465157050444e-11, 3.5693190810581679e-10, 1, 0.0099999992431144356, 0, 0, 0, 1}},
       {{1, 5.4151701490803706e-13, -1.6607571547921101e-10, 0.0030000114865706565,
         -5.4151712468498177e-13, 1, -6.6100540002328695e-10, -0.0020000337446110057,
         1.6607571547885307e-10, 6.6100540002337691e-10, 1, 0.0099999982652406675, 0, 0, 0, 1}}}};
  for (auto method : {RegistrationMethod::point_to_plane, RegistrationMethod::point_to_point,
                      RegistrationMethod::gicp}) {
    auto result =
        register_surfaces(reference.view(), scan.view(), initial, performance::parameters(method));
    if (!result || result.value().convergence() != RegistrationConvergence::converged) {
      return 1;
    }
    const auto& metrics = result.value();
    const auto& pose = metrics.final_transform().matrix();
    for (auto kernel : {CpuKernel::scalar, CpuKernel::automatic, CpuKernel::avx2}) {
      for (unsigned int threads : {1U, 2U, 4U, 8U}) {
        auto options = performance::parameters(method);
        options.thread_count = threads;
        options.cpu_kernel = kernel;
        auto context = RegistrationContext::create(reference.view(), options, scan.size());
        if (kernel == CpuKernel::avx2 && !registration::avx2_available()) {
          passed &= !context;
          continue;
        }
        if (!context) {
          return 1;
        }
        if (method == RegistrationMethod::gicp) {
          auto degenerate = OwnedSurface::create(std::vector<Vec3f>(scan.size(), Vec3f{}), {}, {},
                                                 {}, LengthUnit::millimeter, sf)
                                .value();
          passed &= !context.value().align(degenerate.view(), initial);
          passed &= !RegistrationContext::create(degenerate.view(), options, scan.size());
        }
        for (int repeat = 0; repeat < 3; ++repeat) {
          auto parallel = context.value().align(scan.view(), initial);
          if (!parallel) {
            return 1;
          }
          const auto& p = parallel.value();
          passed &= p.final_transform().matrix() == metrics.final_transform().matrix() &&
                    p.inlier_rmse_mm() == metrics.inlier_rmse_mm() &&
                    p.iterations() == metrics.iterations() &&
                    p.valid_pairs() == metrics.valid_pairs() && p.fitness() == metrics.fitness() &&
                    p.convergence() == metrics.convergence() &&
                    p.translation_delta_mm() == metrics.translation_delta_mm() &&
                    p.rotation_delta_deg() == metrics.rotation_delta_deg();
        }
      }
    }
    auto invalid = performance::parameters(method);
    invalid.cpu_kernel = static_cast<CpuKernel>(255);
    passed &= !RegistrationContext::create(reference.view(), invalid, scan.size());
    invalid.cpu_kernel = CpuKernel::automatic;
    invalid.thread_count = 0;
    passed &= !RegistrationContext::create(reference.view(), invalid, scan.size());

    for (std::size_t entry = 0; entry < pose.size(); ++entry) {
      passed &= std::abs(pose[entry] - baseline[method_index][entry]) < 1e-6;
    }
    passed &= metrics.inlier_rmse_mm() < 1e-6;
    ++method_index;
    passed &= metrics.valid_pairs() == scan.size();
    std::cout << std::setprecision(17) << registration_method_name(method) << ','
              << metrics.iterations() << ',' << metrics.valid_pairs() << ','
              << metrics.inlier_rmse_mm();
    for (const auto element : pose) {
      std::cout << ',' << element;
    }
    std::cout << '\n';
  }
  return passed ? 0 : 1;
}
