# PCAD-HET-003: reusable reference registration context

## Identity
- Milestone: M10. Dependency: PCAD-HET-002.
- Objective: prepare a reference once and register multiple scans without rebuilding its search
  index or GICP covariance. Preserve the existing single-call interface and shared solver.

## Allowed scope
- Public registration context, private backend preparation/solve split, CUDA resource lifetime,
  registration tests/benchmark, export/consumer checks and documentation. No new dependencies.

## Contract
- Move-only context owns a validated reference snapshot, frame, immutable solver parameters,
  selected backend, reference covariance and search resources. Caller reference can be modified
  or destroyed after creation. No implicit cache keyed by caller addresses.
- Synchronous align borrows each scan until return. Millimetres, scan-to-reference poses.
- Explicit maximum logical scan points reserves GPU scratch once; over-capacity calls fail
  without changing the context. CPU uses the same bound. No silent reference rebuild or eviction.
- Parameters, reference and backend are fixed per context: create a new context to change them.
  Auto resolves to CPU; explicit unavailable GPU fails. Same-context operations require caller
  serialization; independent contexts own independent resources. Moved-from align returns error.
- Reference preparation is amortized; scan validation, extraction and GICP covariance remain per
  frame. Identical deterministic accumulation and quality metrics to the single-call solver.
- All public operations use Result and noexcept; runtime/allocation errors cannot escape.

## Exclusions
- InspectionPipeline cache, global caches, automatic backend threshold, CPU parallelism and SIMD.
- New algorithms, GPU covariance/solver and changes to quality gate semantics.

## Acceptance
- All three objectives and both backends: exact reused/fresh metrics, multiple different scans,
  caller reference mutation/destruction, move construction/assignment, independent contexts,
  invalid frame/unit/parameters/backend/capacity and successful reuse after rejected inputs.
- CPU/CUDA Debug/Release tests, Release install/export consumer, formatting and GPU memcheck.
- Reproducible benchmark separates context creation, first and subsequent scans and repeated
  one-shot calls; validate convergence and output equality outside measured regions.
- SDK API/ownership/concurrency documentation and task/roadmap evidence.

## Completion evidence
- Completed 2026-09-13.
- Implemented `RegistrationContext` create/align and out-of-line move/destruction in the existing
  public engine module. The prepared backend retains reference extraction, search and covariance;
  shared optimization arithmetic and ordering are unchanged.
- Windows CPU Debug/Release: 31/31 CTest each; CUDA Debug/Release: 32/32 each. Both Release
  `-VerifyInstall` runs passed export/dependency/runtime audit and a standalone consumer that
  creates, moves, assigns, aligns and destroys CPU/GPU contexts (GPU when available).
- Build commands: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-cuda-debug`, and both Release presets with
  `-VerifyInstall`. Release warnings-as-errors and existing CI/check settings remain enabled.
- Context tests compare all metrics exactly for every method/backend across changing scans,
  smaller batches, immutable caller snapshots/parameters, move construction/assignment, organized
  padding, masked points, independent concurrent contexts, and rejected input followed by reuse.
  Malformed scan revalidation catches mutations made after borrowed SurfaceView construction.
- `cmake --build --preset windows-cuda-release --target format-check`: passed.
- GPU context executable under Compute Sanitizer memcheck with leak-check full, racecheck and
  synccheck, all with `--error-exitcode 1`: zero errors/leaks/race hazards/synchronization errors.
- Optional `pointcloud_ad_context_benchmark` built and passed all convergence/result-equality
  gates. [Protocol and raw data](../benchmarks/PCAD-HET-003.md) cover 5 changing scans at 4,225,
  16,641 and 66,049 points. At 66,049 points GPU point-to-plane total fell 45.1%, GPU GICP 25.3%,
  CPU GICP 18.2%; CPU point-to-point measured about 1.1% slower, so no universal speedup is claimed.
- Configured clang-tidy completed for `src/registration/registration_engine.cpp` and
  `src/backends/pcl/pcl_registration_backend.cpp` using the CPU Debug compile database, without
  compiler, bugprone or clang-analyzer diagnostics. Advisory style/modernize warnings remain;
  no checks were weakened. `git diff --check` passed.
- Remaining limitations: retained reference copies increase host memory; no per-context concurrent
  calls, scan covariance caching, inspection preprocessing cache or automatic backend crossover.
  Measurements/tests used one RTX 5070 Ti. Multi-GPU device switching and runtime/OOM fault
  injection were not exercised; GPU runtime failures may require recreation.
