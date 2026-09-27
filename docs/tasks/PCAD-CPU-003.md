# PCAD-CPU-003: Parallel GICP covariance

## Identity
- Task ID: PCAD-CPU-003
- Milestone: M12
- Objective: Parallel GICP covariance.
- Dependencies: PCAD-CPU-002.

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

- Completed 2026-09-27: CPU Debug/Release 33/33; CUDA Debug/Release 34/34. Format and configured static analysis completed.
- Reference and per-scan covariance jobs use immutable indexes with worker-private neighborhood buffers. Eigen decomposition and neighbor ordering unchanged.
- Exact cross-thread all-method metrics pass. Degenerate covariance jobs are drained and the context successfully handles a later valid scan. Context ownership/move/organized/masked/concurrency suite now exercises four-worker budgets.
- Benchmarks: PCAD-CPU-003.csv and PCAD-CPU-003-100k.csv, four workers, 7/3 samples respectively. Hardware/protocol as PERF-001. No concurrent builds during measurement.
