#include "cuda_correspondence.hpp"
#include "performance_fixture.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

namespace {
using namespace pointcloud_ad;
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
void report(int side, std::string_view method, std::string_view backend, std::string_view stage,
            std::vector<double> times) {
  std::sort(times.begin(), times.end());
  const auto at = [&](double q) {
    return times[static_cast<std::size_t>(std::ceil(q * static_cast<double>(times.size()))) - 1];
  };
  std::cout << side * side << ',' << method << ',' << backend << ',' << stage << ',' << times.size()
            << ',' << at(0.5) << ',' << at(0.95) << '\n';
}
} // namespace
int main(int argc, char** argv) {
  try {
    if (!registration_backend_available(ComputeBackend::gpu))
      throw std::runtime_error("actual GPU required");
    const int side = argc > 1 ? std::stoi(argv[1]) : 65;
    const int runs = argc > 2 ? std::stoi(argv[2]) : 3;
    if (side < 25 || side > 1025 || runs < 3 || runs > 30)
      throw std::invalid_argument("usage: resident_benchmark [side25..1025] [runs3..30]");
    const auto rf = FrameId::create("reference").value(), sf = FrameId::create("scan").value();
    const auto reference = performance::patch(rf, side, false, true),
               scan = performance::patch(sf, side, true, true);
    const auto initial = performance::initial(sf, rf);
    std::cout << std::setprecision(10) << "points,method,backend,stage,samples,p50_ms,p95_ms\n";
    for (auto method : {RegistrationMethod::point_to_plane, RegistrationMethod::point_to_point,
                        RegistrationMethod::gicp}) {
      auto options = performance::parameters(method);
      options.thread_count = 4;
      const auto cpu_oracle =
          register_surfaces(reference.view(), scan.view(), initial, options, ComputeBackend::cpu);
      if (!cpu_oracle)
        throw std::runtime_error("CPU oracle failed");
      for (auto backend : {ComputeBackend::cpu, ComputeBackend::gpu}) {
        std::map<std::string, std::vector<double>> timings;
        for (int run = -1; run < runs; ++run) {
          auto start = Clock::now();
          auto fresh = register_surfaces(reference.view(), scan.view(), initial, options, backend);
          const auto oneshot = elapsed(start);
          if (!fresh || fresh.value().convergence() != RegistrationConvergence::converged ||
              fresh.value().inlier_rmse_mm() > 0.001)
            throw std::runtime_error("registration accuracy gate failed");
          for (std::size_t j = 0; j < 16; ++j)
            if (std::abs(fresh.value().final_transform().matrix()[j] -
                         cpu_oracle.value().final_transform().matrix()[j]) > 1e-5)
              throw std::runtime_error("CPU/GPU pose parity failed");
          if (fresh.value().valid_pairs() != cpu_oracle.value().valid_pairs())
            throw std::runtime_error("pair parity failed");
          start = Clock::now();
          auto prepared =
              RegistrationContext::create(reference.view(), options, scan.size(), backend);
          const auto preparation = elapsed(start);
          if (!prepared)
            throw std::runtime_error(prepared.error().message);
          auto context = std::move(prepared).value();
          for (int frame = 0; frame < 3; ++frame) {
            start = Clock::now();
            auto aligned = context.align(scan.view(), initial);
            const auto duration = elapsed(start);
            if (!aligned ||
                aligned.value().final_transform().matrix() !=
                    fresh.value().final_transform().matrix() ||
                aligned.value().inlier_rmse_mm() != fresh.value().inlier_rmse_mm())
              throw std::runtime_error("context reuse parity failed");
            if (run >= 0)
              timings[frame == 0 ? "first" : "reused"].push_back(duration);
          }
          if (run >= 0) {
            timings["prepare"].push_back(preparation);
            timings["oneshot"].push_back(oneshot);
          }
        }
        for (const auto& [stage, times] : timings)
          report(side, registration_method_name(method), compute_backend_name(backend), stage,
                 times);
      }
    }
    using namespace backends::cuda_backend;
    std::vector<Vec3d> points, normals;
    for (auto p : scan.view().points())
      points.push_back({p.x, p.y, p.z});
    for (auto n : reference.view().normals())
      normals.push_back({n.x, n.y, n.z});
    for (bool gicp : {false, true}) {
      CorrespondenceSearch search(reference.view().points(), scan.size());
      search.prepare_resident(normals);
      if (gicp)
        search.prepare_gicp(12, 0.001);
      search.upload_scan(points, normals);
      const auto before = search.transfer_statistics();
      (void)search.evaluate(initial.matrix(), 0.2, 0.1, !gicp, false);
      (void)search.evaluate(initial.matrix(), 0.2, 0.1, !gicp, true);
      const auto after = search.transfer_statistics();
      if (after.host_to_device_bytes != before.host_to_device_bytes ||
          after.device_to_host_bytes - before.device_to_host_bytes != 2 * sizeof(Equation))
        throw std::runtime_error("resident transfer gate failed");
      std::cerr << "points=" << points.size() << ",gicp=" << gicp
                << ",prepare_and_scan_upload_bytes=" << before.host_to_device_bytes
                << ",prepare_download_bytes=" << before.device_to_host_bytes
                << ",two_evaluations_download_bytes="
                << after.device_to_host_bytes - before.device_to_host_bytes
                << ",device_payload_bytes=" << search.device_storage_bytes() << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
