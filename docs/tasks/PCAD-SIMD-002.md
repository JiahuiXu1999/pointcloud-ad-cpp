# PCAD-SIMD-002: SIMD coordinate transformation

## Identity
- Task ID: PCAD-SIMD-002
- Milestone: M13
- Objective: SIMD coordinate transformation.
- Dependencies: PCAD-SIMD-001.

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

- Completed 2026-09-27: CPU Debug/Release 35/35; CUDA Debug/Release 36/36. Format-check and configured kernel static analysis passed (advisory diagnostics only).
- AVX2 transforms four points at a time in an isolated object target; baseline and AVX2 use strict/non-contracting floating-point grouping. Tail processing calls the scalar kernel.
- Tests cover zero/short/nonmultiple lengths, unaligned and in-place buffers, guard values, all three objectives and 1/2/4/8 workers with scalar/auto/AVX2. Public metrics are exactly equal.
- Benchmark PCAD-SIMD-002.csv: same 4,225-point seven-sample four-worker protocol as CPU-003.
