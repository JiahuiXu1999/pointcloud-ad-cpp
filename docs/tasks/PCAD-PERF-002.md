# PCAD-PERF-002: Numerical contract and scalar regression

## Identity
- Task ID: PCAD-PERF-002
- Milestone: M11
- Objective: Numerical contract and scalar regression.
- Dependencies: PCAD-PERF-001.

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

- Completed 2026-09-27: CPU Debug/Release 32/32, format-check and configured clang-tidy passed (advisory warnings only).
- Captured original scalar metrics in benchmarks/PCAD-PERF-002-scalar.csv before arithmetic changes. Numerical test freezes original transforms with the predeclared 1e-6 budget; all existing tests unchanged.
- The original point-to-plane solver on this shallow curve differs from analytic x translation by 1.06e-5 mm. This is a fixture conditioning observation, not an optimization regression; the regression oracle is the captured pre-optimization solver output.
