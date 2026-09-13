# PCAD-HET-001: heterogeneous registration

## Identity
- Task ID: PCAD-HET-001
- Milestone: M10
- Objective: One backend-neutral registration API exposing three objectives on CPU and optional CUDA.
- Dependencies: PCAD-REL-001.

## Allowed scope
- Registration, pipeline selection/provenance, JSON configuration, build presets, tests, a registration
  benchmark, dependency notices and SDK docs.
- Public API: appended method/device options, a registration entry point, backend availability.
- New dependency: optional CUDA feature under the pinned vcpkg registry (ADR-0010).

## Contract
- Inputs: borrowed normalized surfaces remain alive throughout the synchronous call.
- Outputs: owned Result<RegistrationMetrics>; no device memory or exceptions cross the public API.
- Units/frames: millimetres, radians for solver controls, scan-to-reference transforms.
- Determinism: stable source order and ordered double reduction; CPU/GPU tolerance tested separately.
- Errors: invalid configuration, unsupported/unavailable GPU, invalid geometry, CUDA allocation/kernel
  failures; no silent device fallback for explicit GPU.
- Performance: reference uploaded once per solve; CPU indexed search retained. GPU brute-force is an
  experimental correctness implementation, with no speedup promise.

## Exclusions
- No global registration, NDT, GPU preprocessing, unmeasured SIMD, or industrial-accuracy claims.
- Public headers remain backend neutral; all PCL/Eigen includes stay under src/backends/pcl/.

## Acceptance
- Unit tests: known nonidentity rigid motion across all objectives, frames/units/options, missing
  normals, unavailable GPU, degenerate inputs, repeatability, CPU/GPU numeric comparison.
- Integration: optional configuration defaults and JSON round-trip, pipeline execution provenance.
- Builds: Windows Debug; Windows Release with install verification; CUDA-enabled Debug/Release.
- Documentation: ADR, SDK execution/metric semantics, roadmap, current milestone and task evidence.

## Completion evidence
- Commands run:
  - `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`
  - `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`
  - `pwsh ./scripts/build.ps1 -Preset windows-cuda-debug`
  - `pwsh ./scripts/build.ps1 -Preset windows-cuda-release -VerifyInstall`
  - `cmake --build --preset windows-msvc-debug --target format-check`
  - `cmake --build --preset windows-cuda-release --target pointcloud_ad_registration_benchmark`
  - `compute-sanitizer --tool memcheck --error-exitcode 1 --leak-check full ./out/build/windows-cuda-release/bin/pointcloud_ad_heterogeneous_tests.exe`
  - `compute-sanitizer --tool racecheck --error-exitcode 1 ./out/build/windows-cuda-release/bin/pointcloud_ad_heterogeneous_tests.exe`
  - `clang-tidy -p out/build/windows-msvc-debug src/registration/registration_engine.cpp src/backends/pcl/pcl_registration_backend.cpp src/pipeline/inspection_pipeline.cpp`
- Test results (2026-09-13): all four presets passed 29/29 CTest entries. Both Release installations
  passed export/runtime audits and standalone consumers. CUDA tests required and used the RTX 5070 Ti.
  GPU memcheck: zero errors, zero leaked bytes; racecheck: zero hazards/errors/warnings. Formatting
  passed. Repository-configured clang-tidy completed without compiler, bugprone or clang-analyzer
  findings in the checked translation units; advisory naming/designated-initializer/style warnings
  and pre-existing redundant moves remain, so this is not an all-checks-zero-warning tidy claim.
- Benchmark: all 18 method/device/size rows passed convergence and RMSE gates. Timings and limits
  are recorded in `docs/benchmarks/PCAD-HET-001.md` and its CSV. No production speedup is promised.
- Environment: Windows, MSVC 19.51, CUDA 13.2.78, RTX 5070 Ti, NVIDIA driver 616.64. Linux/Sanitizer
  CI presets were not rerun locally on this Windows host; NVIDIA device checks above are distinct
  from CPU ASan/UBSan. The existing CI gates have not been weakened.
- Remaining risks: GPU exact search is quadratic; industrial samples and GPU performance tuning remain.
