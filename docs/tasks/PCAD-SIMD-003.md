# PCAD-SIMD-003: SIMD objective arithmetic

## Identity
- Task ID: PCAD-SIMD-003
- Milestone: M13
- Objective: SIMD objective arithmetic.
- Dependencies: PCAD-SIMD-002.

## Allowed scope
- Files/modules: relevant benchmarks, registration/backend helpers, tests, CMake and documentation.
- Public API changes: explicit execution controls where required; no third-party types.
- New dependencies: none.

## Contract
- Inputs borrowed synchronously; contexts own prepared reference and work resources.
- Outputs own results; millimetres, right-handed scan-to-reference transforms.
- Determinism: stable point/tie ordering; see PERFORMANCE_PLAN.md and numerical contract.
- Error cases: invalid options, allocation and worker failures return Result at public boundaries.
- Performance budget: compare Release complete calls; report regressions honestly.

## Exclusions
- GPU-resident optimization, inspection caches and new registration algorithms.
- No changes to inspection status/exit-code semantics or dependency boundaries.

## Acceptance
- Tests before/with behavior changes; existing CPU Debug/Release tests pass.
- Required presets: windows-msvc-debug, windows-msvc-release; CUDA regression for shared solver changes.
- Install consumer for public API changes; formatting and configured static analysis.
- Documentation: evidence, roadmap, numerical/SDK contract and benchmark report as applicable.

## Completion evidence
- Status: Complete. Verification evidence follows.
- Remaining risks: industrial acceptance and cross-hardware performance are separate gates.

- Completed 2026-09-27: CPU Debug/Release 35/35 and CUDA Debug/Release 36/36; Release installed consumers passed for both backends. CPU Release `-VerifyInstall -VerifyPackage` passed, including CPack ZIP extraction, export/path audit and consumer/CLI with only SDK + Windows runtime paths.
- Scalar/automatic/AVX2 exact-output matrix, 1/2/4/8 workers, disabled-AVX2 build, finite-difference objective tests, Huber boundaries and worker-error recovery passed.
- Format-check and configured static analysis completed; no outstanding production-code compiler/bugprone/analyzer failures. Compute Sanitizer memcheck (including leak check), racecheck and synccheck report zero errors/leaks/hazards.
- Scalar and AVX2 objective kernels share operation grouping; row-major contributions avoid scatter, point-to-point skips identity information products. Automatic point-to-plane objective remains scalar based on kernel measurements, while transforms and other objectives use supported AVX2.
- Final sequential benchmarks cover 4,225 / 100,489 points at 1/4 workers with scalar/auto, and 1,000,000 points at four workers including all three registrations and full inspection. All correctness gates passed. Original baseline comparison and the absence of stable end-to-end SIMD-only speedup are documented in [the final report](../benchmarks/PCAD-SIMD-003.md).
- Public parameter layout changed; SDK consumers must rebuild. Linux/cross-hardware and industrial acceptance remain explicitly unverified; M14-M16 remain future work.
