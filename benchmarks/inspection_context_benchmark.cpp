#include "performance_fixture.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <pointcloud_ad/inspection_context.hpp>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
  using namespace pointcloud_ad;
  using Clock = std::chrono::steady_clock;
  const auto elapsed = [](Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  };
  try {
    const int side = argc > 1 ? std::stoi(argv[1]) : 65;
    const int runs = argc > 2 ? std::stoi(argv[2]) : 3;
    const unsigned int threads = argc > 3 ? static_cast<unsigned int>(std::stoul(argv[3])) : 4;
    if (side < 25 || side > 1025 || runs < 3 || runs > 30 || threads < 1 || threads > 64)
      throw std::invalid_argument("invalid benchmark size/runs/threads");
    const auto rf = FrameId::create("fixture").value(), sf = FrameId::create("scanner").value();
    const bool defect = argc > 4 && std::string(argv[4]) == "defect";
    const auto reference = performance::patch(rf, side, false, false);
    auto scan = performance::patch(sf, side, false, false);
    if (defect) {
      std::vector<Vec3f> points(scan.view().points().begin(), scan.view().points().end());
      std::vector<Vec3f> normals(scan.view().normals().begin(), scan.view().normals().end());
      for (auto& point : points)
        point.z +=
            static_cast<float>(0.7 * std::exp(-(point.x * point.x + point.y * point.y) / 4.5));
      scan = OwnedSurface::create(std::move(points), std::move(normals), {}, {},
                                  LengthUnit::millimeter, sf)
                 .value();
    }
    std::cout << std::setprecision(10) << "points,backend,threads,stage,samples,p50_ms,p95_ms\n";
    for (auto backend : {ComputeBackend::cpu, ComputeBackend::gpu}) {
      if (!registration_backend_available(backend))
        continue;
      auto config = performance::make_config();
      config.execution.backend = backend;
      config.execution.thread_count = threads;
      auto pipeline = InspectionPipeline::create(config);
      if (!pipeline)
        throw std::runtime_error(pipeline.error().message);
      std::map<std::string, std::vector<double>> timings;
      for (int run = -1; run < runs; ++run) {
        auto start = Clock::now();
        auto fresh = pipeline.value().run(reference.view(), scan.view());
        const auto oneshot = elapsed(start);
        if (!fresh || fresh.value().verdict != (defect ? Verdict::fail : Verdict::pass) ||
            fresh.value().regions.empty() == defect || !fresh.value().registration.final_pose)
          throw std::runtime_error("fresh inspection failed");
        const auto expected_pose = fresh.value().registration.final_pose;
        if (!expected_pose)
          throw std::runtime_error("fresh pose absent");
        const auto expected_matrix = expected_pose->matrix();
        start = Clock::now();
        auto prepared = InspectionContext::create(config, reference.view(), scan.size());
        const auto prepare = elapsed(start);
        if (!prepared)
          throw std::runtime_error(prepared.error().message);
        for (int frame = 0; frame < 3; ++frame) {
          start = Clock::now();
          auto result = prepared.value().run(scan.view());
          const auto duration = elapsed(start);
          if (!result)
            throw std::runtime_error("cached inspection failed");
          const auto actual_pose = result.value().registration.final_pose;
          if (!actual_pose)
            throw std::runtime_error("cached pose absent");
          if (result.value().verdict != fresh.value().verdict ||
              result.value().regions.size() != fresh.value().regions.size() ||
              !result.value().registration.final_pose || actual_pose->matrix() != expected_matrix ||
              result.value().deviations.rms_mm != fresh.value().deviations.rms_mm ||
              result.value().coverage.coverage_ratio != fresh.value().coverage.coverage_ratio)
            throw std::runtime_error("inspection context parity failed");
          if (run >= 0) {
            timings[frame == 0 ? "first" : "reused"].push_back(duration);
            if (frame > 0)
              for (const auto& timing : result.value().timings)
                timings["reused_stage_" + std::to_string(static_cast<unsigned int>(timing.stage))]
                    .push_back(static_cast<double>(timing.duration_us) / 1000.0);
          }
        }
        if (run >= 0) {
          timings["oneshot"].push_back(oneshot);
          timings["prepare"].push_back(prepare);
        }
      }
      for (auto& [stage, times] : timings) {
        std::sort(times.begin(), times.end());
        const auto at = [&](double q) {
          return times[static_cast<std::size_t>(std::ceil(q * static_cast<double>(times.size()))) -
                       1];
        };
        std::cout << side * side << ',' << compute_backend_name(backend) << ',' << threads << ','
                  << stage << ',' << times.size() << ',' << at(0.5) << ',' << at(0.95) << '\n';
      }
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
