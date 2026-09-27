# PCAD-SIMD-001: Runtime scalar and AVX2 dispatch

## Identity
- Task ID: PCAD-SIMD-001
- Milestone: M13
- Objective: Runtime scalar and AVX2 dispatch.
- Dependencies: PCAD-CPU-003.

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

- Completed 2026-09-27: CPU Debug/Release 34/34; CUDA Debug/Release 35/35; installed consumer, format-check and configured dispatch static analysis passed.
- Baseline dispatch checks CPUID and OS XMM/YMM state. Tests explicitly reject missing feature/state bits and forced unsupported selection. ISA compile options are isolated in a private interface target for the next arithmetic task.
- SIMD arithmetic is introduced in SIMD-002/003; this task adds selection/validation only. One transient Windows artifact file-lock failure passed on retry.
