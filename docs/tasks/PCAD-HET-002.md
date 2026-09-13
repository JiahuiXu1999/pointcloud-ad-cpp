# PCAD-HET-002: exact indexed CUDA correspondence search

## Identity
- Task ID: PCAD-HET-002
- Milestone: M10
- Objective: Replace the default quadratic CUDA scan with a deterministic spatial index without
  changing objective/device selection, exact nearest correspondence, or quality-gate semantics.
- Dependencies: PCAD-HET-001.

## Allowed scope
- Private CUDA search/index construction, direct search tests, benchmarks, CMake and architecture docs.
- Public API changes: none. Private brute-force selection remains available for verification only.
- New dependencies: none.

## Contract
- Inputs: finite millimetre float coordinates, owned by the caller until the synchronous constructor
  or query returns. The search owns a copy of reference/index/device storage for its lifetime.
- Outputs: original compact-reference indices; -1 when no point lies within the inclusive radius.
- Units/frames: squared distances in mm^2; both inputs must share a frame before reaching this adapter.
- Determinism: stable original-index tie breaking, deterministic median partition and node layout;
  identical indices to exhaustive double-distance search over the same float coordinates.
- Errors: empty/oversized reference, invalid coordinates/radius, capacity/device/runtime failures.
- Performance budget: O(N) index/device memory, balanced host construction, pruned stackless GPU
  traversal. Worst-case search remains linear per query; no universal logarithmic guarantee.

## Exclusions
- No changes to CPU PCL search, GICP objective, automatic device selection, SIMD or thread policy.
- No GPU index construction, cross-inspection cache, approximate-nearest search or new dependency.
- PCL/Eigen stay within the existing PCL backend; no CUDA types enter public headers.

## Acceptance
- CPU index tests: permutation, bounds, leaf and escape invariants, determinism, degenerate geometry.
- GPU tests: exhaustive oracle on random/nonuniform/duplicate/planar/large-coordinate clouds; exact
  ties, inclusive thresholds, empty and malformed queries, changing query batches/radii; old tiled
  kernel independently agrees. All three registration objectives remain CPU/GPU compatible.
- Builds: Windows CPU Debug and Release; CUDA Debug and Release, warnings-as-errors in Release.
- GPU tools: memcheck/leakcheck, racecheck and synchronization checks on direct search tests.
- Performance: reproducible indexed-vs-tiled microbenchmark plus larger indexed workloads and
  existing end-to-end registration benchmark; include construction/transfers in measurements.
- Documentation: task evidence, roadmap, architecture/SDK current implementation and benchmark limits.

## Completion evidence
- Completed 2026-09-13 on Intel Core i5-13400F / RTX 5070 Ti, MSVC 19.51, CUDA 13.2.78.
- `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`: 30/30 CTest passed.
- `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`: 30/30 passed;
  export/dependency/runtime audit and standalone installed consumer passed.
- `pwsh ./scripts/build.ps1 -Preset windows-cuda-debug`: 31/31 passed.
- `pwsh ./scripts/build.ps1 -Preset windows-cuda-release -VerifyInstall`: 31/31 passed;
  export/dependency/runtime audit and standalone installed consumer passed.
- Release builds retain warnings-as-errors. CPU builds require no CUDA headers or runtime.
- `cmake --build --preset windows-cuda-release --target format-check`: passed.
- `clang-tidy -p out/build/windows-msvc-debug tests/unit/spatial_index_test.cpp`, also with
  `--header-filter=.*spatial_index.hpp`: completed without compiler, bugprone or analyzer
  diagnostics. Advisory naming, modernize, test-constant and pragma-once warnings remain;
  this is not a claim that every advisory style check is clean. No checks were weakened.
- `compute-sanitizer --tool memcheck --error-exitcode 1 --leak-check full
  ./out/build/windows-cuda-release/bin/pointcloud_ad_cuda_search_tests.exe`: zero errors/leaks.
- Same executable with `--tool racecheck --error-exitcode 1`: zero errors/warnings/hazards;
  with `--tool synccheck --error-exitcode 1`: zero errors.
- Final CUDA search tests include uneven 3D clusters and isolated outliers. Both kernels agree
  exactly with the CPU exhaustive double oracle, including stable original-index ties. Existing
  three-objective CPU/GPU pose/metric and quality-gate regression tests remain passing.
- Upload/query exception paths synchronize submitted work before host buffers leave scope.
- Optional search and registration benchmark targets built and completed their correctness gates.
  [Measurement protocol and raw CSV](../benchmarks/PCAD-HET-002.md): at 16,384 lattice points,
  indexed search is 0.1613 ms versus tiled 11.7971 ms; full context lifetime is 3.5165 ms versus
  12.3804 ms. Million-point lattice/plane queries take 9.381/8.9656 ms, preparation
  312.707/219.815 ms, with 37 MiB allocated device payload. Complete 4,225-point point-to-plane
  registration is 14.0867 ms CPU and 2.4421 ms indexed GPU on the synthetic fixture.
- Implementation and documentation reviewed; `git diff --check` passed.
- Remaining risks: host index preparation, traversal divergence and degenerate distributions can
  limit gains; real industrial acceptance and automatic dispatch thresholds remain separate work.
