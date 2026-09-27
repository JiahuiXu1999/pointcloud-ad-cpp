#include "cpu_dispatch.hpp"
#include "performance_fixture.hpp"
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

// Psapi requires the Windows declarations above. Keep it in a separate include block.
#include <psapi.h>
#else
#include <sys/resource.h>
#endif

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
double elapsed(Clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}
double peak_process_mib() {
#ifdef _WIN32
  PROCESS_MEMORY_COUNTERS counters{};
  if (!GetProcessMemoryInfo(GetCurrentProcess(), &counters, static_cast<DWORD>(sizeof(counters)))) {
    return -1;
  }
  return static_cast<double>(counters.PeakWorkingSetSize) / (1024.0 * 1024.0);
#else
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    return -1;
  }
#ifdef __APPLE__
  return static_cast<double>(usage.ru_maxrss) / (1024.0 * 1024.0);
#else
  return static_cast<double>(usage.ru_maxrss) / 1024.0;
#endif
#endif
}
void report(int side, std::string_view method, std::string_view stage,
            std::vector<double> samples) {
  std::sort(samples.begin(), samples.end());
  const auto percentile = [&](double q) {
    return samples[static_cast<std::size_t>(std::ceil(q * static_cast<double>(samples.size()))) -
                   1];
  };
  std::cout << side * side << ',' << method << ',' << stage << ',' << samples.size() << ','
            << percentile(0.5) << ',' << percentile(0.95) << ',' << peak_process_mib() << '\n';
}
} // namespace

int main(int argc, char** argv) {
  try {
    const int side = argc > 1 ? std::stoi(argv[1]) : 65;
    const int runs = argc > 2 ? std::stoi(argv[2]) : 7;
    const int threads = argc > 3 ? std::stoi(argv[3]) : 1;
    const std::string kernel_name = argc > 4 ? argv[4] : "auto";
    CpuKernel requested = CpuKernel::automatic;
    if (kernel_name == "scalar") {
      requested = CpuKernel::scalar;
    } else if (kernel_name == "avx2") {
      requested = CpuKernel::avx2;
    } else if (kernel_name != "auto") {
      throw std::invalid_argument("kernel must be auto, scalar or avx2");
    }
    const auto resolved = registration::select_cpu_kernel(requested);
    std::cerr << "threads=" << threads << ",transform_kernel=" << cpu_kernel_name(resolved)
              << ",objective_policy=" << kernel_name << ",pipeline_kernel=auto\n";

    if (threads < 1 || threads > 64) {
      throw std::invalid_argument("threads must be in [1,64]");
    }
    if (side < 25 || side > 1025 || runs < 3 || runs > 100) {
      throw std::invalid_argument("usage: benchmark [side 25..1025] [samples 3..100]");
    }
    const auto reference_frame = FrameId::create("fixture").value();
    const auto scan_frame = FrameId::create("scanner").value();
    const auto initial = performance::initial(scan_frame, reference_frame);
    std::cout << std::setprecision(10)
              << "points,method,stage,samples,p50_ms,p95_ms,cumulative_peak_process_mib\n";
    const auto reference = performance::patch(reference_frame, side, false, true);
    const auto scan = performance::patch(scan_frame, side, true, true);
    for (auto method : {RegistrationMethod::point_to_plane, RegistrationMethod::point_to_point,
                        RegistrationMethod::gicp}) {
      auto options = performance::parameters(method);
      options.thread_count = static_cast<std::uint32_t>(threads);
      options.cpu_kernel = requested;
      std::map<std::string, std::vector<double>> timings;
      for (int run = -1; run < runs; ++run) {
        auto start = Clock::now();
        auto fresh = register_surfaces(reference.view(), scan.view(), initial, options);
        const double one_ms = elapsed(start);
        if (!fresh || fresh.value().convergence() != RegistrationConvergence::converged ||
            fresh.value().inlier_rmse_mm() > 0.001) {
          throw std::runtime_error("registration accuracy gate failed");
        }
        start = Clock::now();
        auto prepared = RegistrationContext::create(reference.view(), options, scan.size());
        const double prepare_ms = elapsed(start);
        if (!prepared) {
          throw std::runtime_error(prepared.error().message);
        }
        auto context = std::move(prepared).value();
        for (int frame = 0; frame < 3; ++frame) {
          start = Clock::now();
          auto aligned = context.align(scan.view(), initial);
          const double align_ms = elapsed(start);
          if (!aligned ||
              aligned.value().final_transform().matrix() !=
                  fresh.value().final_transform().matrix() ||
              aligned.value().inlier_rmse_mm() != fresh.value().inlier_rmse_mm()) {
            throw std::runtime_error("reused context differs");
          }
          if (run >= 0) {
            timings[frame == 0 ? "first" : "reused"].push_back(align_ms);
          }
        }
        if (run >= 0) {
          timings["oneshot"].push_back(one_ms);
          timings["prepare"].push_back(prepare_ms);
        }
      }
      for (const auto& [stage, samples] : timings) {
        report(side, registration_method_name(method), stage, samples);
      }
    }
    if (argc > 5 && std::string(argv[5]) == "registration-only") {
      return 0;
    }
    // Full inspection uses a fixed-spacing plane so increasing size does not inflate neighborhood
    // density.
    const auto flat_reference = performance::patch(reference_frame, side, false, false);
    const auto flat_scan = performance::patch(scan_frame, side, false, false);
    auto config = performance::make_config();
    config.execution.thread_count = static_cast<std::uint32_t>(threads);
    auto pipeline = InspectionPipeline::create(config);
    if (!pipeline) {
      throw std::runtime_error(pipeline.error().message);
    }
    std::map<std::string, std::vector<double>> timings;
    for (int run = -1; run < runs; ++run) {
      const auto start = Clock::now();
      auto result = pipeline.value().run(flat_reference.view(), flat_scan.view());
      const double total_ms = elapsed(start);
      if (!result || result.value().verdict != Verdict::pass || !result.value().regions.empty()) {
        throw std::runtime_error("full inspection correctness gate failed");
      }
      if (run >= 0) {
        timings["total"].push_back(total_ms);
        for (const auto& timing : result.value().timings) {
          timings["stage_" + std::to_string(static_cast<unsigned int>(timing.stage))].push_back(
              static_cast<double>(timing.duration_us) / 1000.0);
        }
      }
    }
    for (const auto& [stage, samples] : timings) {
      report(side, "inspection", stage, samples);
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
