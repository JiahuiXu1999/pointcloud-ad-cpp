# PCAD-CPU-001: Bounded executor and parallel queries

## Identity
- Task ID: PCAD-CPU-001
- Milestone: M12
- Objective: Bounded executor and parallel queries.
- Dependencies: PCAD-PERF-003.

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

- Completed 2026-09-27: CPU Debug/Release 33/33; CUDA Debug/Release 34/34. CPU Release -VerifyInstall passed with execution option in installed consumer.
- Executor tests cover block edges, budgets, repeated dispatch, deterministic exception selection, job draining and reuse. Numerical fixture exactly matches at budgets 1/2/4/8 for every objective.
- Pinned PCL KdTreeFLANN and FLANN KDTreeSingleIndex query implementations inspected: queries use caller/local scratch, immutable index, FLANN default cores=1. No index mutation occurs during jobs.
- format-check and configured clang-tidy completed; advisory diagnostics remain, including intentional rejection-test empty catch.
- Seven-sample four-worker benchmark: benchmarks/PCAD-CPU-001.csv. Full inspection remains preprocessing-bound; no universal speedup claim.
