# PCAD-CPU-002: Fixed-block objective and metric reduction

## Identity
- Task ID: PCAD-CPU-002
- Milestone: M12
- Objective: Fixed-block objective and metric reduction.
- Dependencies: PCAD-CPU-001.

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

- Completed 2026-09-27: CPU Debug/Release 33/33; CUDA Debug/Release 34/34. Format-check and configured backend static analysis completed.
- Fixed 256-point blocks accumulate independently and merge by block index for both objective equations and final RMSE/counts. Cache-line padding is explicit; warnings-as-errors remains enabled.
- Numerical tests remain exactly equal at 1/2/4/8 threads and within original scalar regression budget; existing quality-gate/acceptance tests unchanged.
- Four-worker seven-sample benchmark passed: benchmarks/PCAD-CPU-002.csv.
