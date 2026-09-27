# PCAD-PERF-001: End-to-end inspection and registration baseline

## Identity
- Task ID: PCAD-PERF-001
- Milestone: M11
- Objective: End-to-end inspection and registration baseline.
- Dependencies: PCAD-HET-003.

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

- Completed 2026-09-27: CPU Debug/Release 31/31; format-check passed.
- Configured clang-tidy on pipeline and benchmark completed without compiler/analyzer errors; advisory style warnings remain.
- Release performance benchmark passed at sides 65 (7 samples) and 317 (3 samples), repeated without concurrent builds/analysis. See benchmarks/PCAD-PERF-001.md and CSV.
- Comparison stage timing now covered by the pipeline test. Data generation and correctness checks excluded from timing.
