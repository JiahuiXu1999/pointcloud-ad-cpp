# M11-M13 verification and performance

Measured on 2026-09-27. M11 adds the baseline, numerical contract and reusable iteration buffers; M12 adds a context-owned bounded executor, fixed-block reductions and parallel GICP covariance; M13 adds isolated scalar/AVX2 kernels with runtime dispatch. No new dependencies were introduced.

## Protocol

See [M11 baseline](PCAD-PERF-001.md) for fixtures, hardware and percentile semantics. Intel Core i5-13400F (10 cores / 16 logical processors), MSVC 19.51, Windows x64 Release. GPU regression used RTX 5070 Ti, driver 616.64 and CUDA 13.2.78; all performance tables below use CPU correspondence search.

The benchmark arguments are side length, measured runs, worker budget (1..64), arithmetic (`auto`, `scalar`, `avx2`) and optional `registration-only`. One warm-up run is excluded. Each prepared context processes three identical scans: first timing has one sample per run; reused timing has two. Explicit arithmetic selection affects registration only; full inspection always uses automatic arithmetic. Metadata is printed on stderr. Runs are sequential without concurrent builds, analysis or other project benchmarks.

One-shot includes context creation, solve and destruction. Reused timings exclude preparation. Synthetic curved registration uses a known translation and supplied normals; the accuracy gate requires convergence and RMSE <= 0.001 mm. Fresh/reused pose and RMSE must match exactly. Flat full inspection includes CPU preprocessing/comparison/detection and report construction, excludes file I/O/serialization, and must PASS without regions. Generating fixtures and checking correctness are outside timing.

CSV P50/P95 are nearest-rank percentiles. Three runs at 100k/1M do not establish a reliable tail estimate. The new memory column is cumulative process peak working set (MiB), not a per-stage allocation; -1 means unavailable. It includes fixtures and all preceding cases. Baseline CSVs did not record memory, so no memory improvement ratio is inferred.

## End-to-end results

P50 milliseconds. Baseline is the original one-worker solver before M11-M13. Final is four workers with automatic arithmetic. Improvement is `(baseline - final) / baseline`; negative values are regressions. This measures the combined implementation and worker-budget change, not SIMD alone.

| Points | Operation | Baseline | Final | Time reduction |
|---:|---|---:|---:|---:|
| 4,225 | point_to_plane oneshot | 12.672 | 11.264 | 11.1% |
| 4,225 | point_to_plane reused | 11.901 | 10.207 | 14.2% |
| 4,225 | point_to_point oneshot | 12.817 | 11.326 | 11.6% |
| 4,225 | point_to_point reused | 12.467 | 9.983 | 19.9% |
| 4,225 | gicp oneshot | 25.526 | 15.757 | 38.3% |
| 4,225 | gicp reused | 18.483 | 12.551 | 32.1% |
| 4,225 | inspection total | 34.989 | 37.162 | -6.2% |
| 100,489 | point_to_plane oneshot | 343.937 | 248.763 | 27.7% |
| 100,489 | point_to_plane reused | 323.441 | 231.950 | 28.3% |
| 100,489 | point_to_point oneshot | 357.068 | 245.564 | 31.2% |
| 100,489 | point_to_point reused | 333.023 | 229.229 | 31.2% |
| 100,489 | gicp oneshot | 656.557 | 363.686 | 44.6% |
| 100,489 | gicp reused | 496.591 | 288.148 | 42.0% |
| 100,489 | inspection total | 901.441 | 849.608 | 5.7% |

Small full inspection regresses with four workers on this fixture, so the SDK's default worker budget remains 1. At 100k, preprocessing alone takes about 547 ms of the final 850 ms inspection: registration optimization cannot remove this cost. Comparison timings also vary, and no inspection-wide parallel speedup is claimed. M15 should prioritize measured preprocessing/comparison bottlenecks and reference reuse.

## Isolating SIMD from worker count

Four-worker one-shot P50, same final executable. These runs use separate processes and short sequential samples, so small differences can include scheduler/frequency/memory noise.

| Points | Method | Forced scalar ms | Automatic ms | Time reduction |
|---:|---|---:|---:|---:|
| 4,225 | point_to_plane | 11.295 | 11.264 | 0.3% |
| 4,225 | point_to_point | 11.477 | 11.326 | 1.3% |
| 4,225 | gicp | 16.382 | 15.757 | 3.8% |
| 100,489 | point_to_plane | 248.420 | 248.763 | -0.1% |
| 100,489 | point_to_point | 239.511 | 245.564 | -2.5% |
| 100,489 | gicp | 349.050 | 363.686 | -4.2% |

The end-to-end matrix does **not** demonstrate a stable additional SIMD speedup. Keep the explicit scalar control and compare production workloads before selecting a deployment policy. The substantial baseline gain above comes primarily from CPU parallel work and reuse.

The independent 100,000-element kernel benchmark (one warm-up, seven timed runs, full output checksum outside timing) explains the automatic policy:

| Kernel | Scalar P50 ms | AVX2 P50 ms | Time reduction |
|---|---:|---:|---:|
| transform | 0.1515 | 0.0966 | 36.2% |
| point_to_plane | 2.5140 | 2.6684 | -6.1% |
| point_to_point | 3.6558 | 3.3146 | 9.3% |
| gicp | 4.9493 | 4.3853 | 11.4% |

Scalar/AVX2 output checksums match; unit tests additionally require exact full-array contributions. Automatic point-to-plane objectives stay scalar because their vector form was slower on this CPU; coordinate transforms and point-to-point/GICP objectives use AVX2 when compiled and supported. Forced AVX2 covers all kernels for comparison. This conservative per-method policy is based on this host, not an adaptive cross-hardware policy. M16 must validate wider hardware/workloads.

## Million-point scale validation

Four workers, automatic arithmetic, 1,000,000 points per surface, three measured runs after warm-up. All accuracy/reuse/full-inspection gates passed. No corresponding M11 million-point registration/full-inspection baseline was collected, so no speedup ratio is claimed.

| Operation | P50 ms | P95 ms | Cumulative process peak MiB |
|---|---:|---:|---:|
| point_to_plane oneshot | 2357.222 | 2457.106 | 270.4 |
| point_to_plane reused | 2123.699 | 2298.743 | 270.4 |
| point_to_point oneshot | 2325.360 | 2396.995 | 270.4 |
| point_to_point reused | 2140.096 | 2480.468 | 270.4 |
| gicp oneshot | 3579.578 | 3629.994 | 470.4 |
| gicp reused | 2873.446 | 2918.714 | 470.4 |
| inspection total | 8425.475 | 8671.139 | 470.4 |

Raw data: [million-point measurement](PCAD-SIMD-003-1m.csv). The peak working set is cumulative for the benchmark process and includes earlier cases, not just the final inspection.

## Validation

- Windows CPU Debug and Release: **35/35 tests passed** per configuration.
- Windows CUDA Debug and Release: **36/36 tests passed** per configuration; Release installed consumers passed for CPU and CUDA.
- AVX2-disabled CPU Debug build: **35/35 passed**, runtime reported unavailable; the enabled build was restored and passed 35/35 again.
- Numerical regression: all three objectives, worker budgets 1/2/4/8, scalar/automatic/available AVX2, repeated contexts and all public pose/metric values exactly equal. The frozen pre-optimization scalar fixture stays within its declared 1e-6 budget.
- Kernel tests cover zero/short/tail lengths, non-aligned input/output, in-place transforms, guards, Huber boundary values, independent finite-difference objective derivatives, SPD GICP information and point-to-point identity handling.
- Executor tests cover exact-once blocks, deterministic lowest-block error selection, draining and reuse. Context tests exercise varying scans, masks, organized inputs, ownership/moves and concurrent independent contexts. Degenerate covariance failures return Result errors and allow subsequent reuse.
- Compute Sanitizer on the CUDA registration-context executable: memcheck **0 errors / 0 leaks**, racecheck **0 hazards / 0 errors / 0 warnings**, synccheck **0 errors**.
- Format-check passed. Configured clang-tidy completed on changed kernels/backend/benchmarks and relevant tests; existing/advisory style diagnostics remain, with no outstanding compiler, bugprone or analyzer errors in the final production-code check.
- Compile-command audit finds `/arch:AVX2` only on `cpu_kernels_avx2.cpp`; scalar/dispatch and public consumers have no new AVX2 requirement. Strict/noncontracting kernel arithmetic preserves operation grouping.

Public `RegistrationParameters` layout changed; rebuild downstream C++ consumers with matching headers and DLL. No public point-cloud layout or JSON schema change. Same-context calls remain serialized by the caller; independent contexts have independent thread budgets. See [numerical contract](../architecture/NUMERICAL_REPRODUCIBILITY.md) and [SDK controls](../SDK.md).

## Reproduction

Run from the repository root in PowerShell. Dot-sourcing the build script preserves its initialized compiler environment for the opt-in targets:

```powershell
. ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall -VerifyPackage
cmake --build --preset windows-msvc-release --target pointcloud_ad_performance_benchmark pointcloud_ad_kernel_benchmark
./out/build/windows-msvc-release/bin/pointcloud_ad_performance_benchmark.exe 65 7 4 auto
./out/build/windows-msvc-release/bin/pointcloud_ad_performance_benchmark.exe 317 3 4 auto
./out/build/windows-msvc-release/bin/pointcloud_ad_performance_benchmark.exe 317 3 4 scalar registration-only
./out/build/windows-msvc-release/bin/pointcloud_ad_performance_benchmark.exe 1000 3 4 auto
./out/build/windows-msvc-release/bin/pointcloud_ad_kernel_benchmark.exe
pwsh ./scripts/build.ps1 -Preset windows-msvc-debug
pwsh ./scripts/build.ps1 -Preset windows-cuda-debug
pwsh ./scripts/build.ps1 -Preset windows-cuda-release -VerifyInstall
cmake --build --preset windows-msvc-debug --target format-check
```

To test the portable build, configure `windows-msvc-debug` with `-DPOINTCLOUDAD_ENABLE_AVX2=OFF`, run its build/tests, then restore `ON`. Compute Sanitizer commands use `--tool memcheck --leak-check full`, `--tool racecheck` and `--tool synccheck`, each with `--error-exitcode 1`, against `out/build/windows-cuda-debug/bin/pointcloud_ad_context_tests.exe`.

## Raw measurements and limits

- Original baseline: [4,225 points](PCAD-PERF-001-baseline.csv), [100,489 points](PCAD-PERF-001-100k.csv), [frozen numerical oracle](PCAD-PERF-002-scalar.csv).
- Final 4,225 points: [1/scalar](PCAD-SIMD-003-65-1-scalar.csv), [1/auto](PCAD-SIMD-003-65-1-auto.csv), [4/scalar](PCAD-SIMD-003-65-4-scalar.csv), [4/auto](PCAD-SIMD-003-65-4-auto.csv).
- Final 100,489 points: [1/scalar](PCAD-SIMD-003-317-1-scalar.csv), [1/auto](PCAD-SIMD-003-317-1-auto.csv), [4/scalar](PCAD-SIMD-003-317-4-scalar.csv), [4/auto](PCAD-SIMD-003-317-4-auto.csv).
- [Kernel measurements](PCAD-SIMD-003-kernels.csv).

These are synthetic measurements with supplied normals on one Windows host. Linux builds, wider compiler/CPU matrices, industrial data, production thresholds, transfer profiling and industrial cycle times were not newly validated in M11-M13. There is no cross-compiler/device bitwise guarantee. GPU-resident optimization remains M14; complete inspection caching/parallel optimization remains M15; deployment tuning and industrial acceptance remain M16.
