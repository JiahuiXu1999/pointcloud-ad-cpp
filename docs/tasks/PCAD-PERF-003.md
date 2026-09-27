# PCAD-PERF-003: Reusable registration workspace

## Identity
- Task ID: PCAD-PERF-003
- Milestone: M11
- Objective: Reusable registration workspace.
- Dependencies: PCAD-PERF-002.

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

- Completed 2026-09-27: CPU Debug/Release 32/32, CUDA Debug/Release 33/33; format-check and configured backend clang-tidy passed (advisory warnings only).
- Existing changing-size/move/recovery context tests remain exact. New CUDA query_into oracle tests verify retained host output allocation across normal/short/empty calls.
- Compute Sanitizer memcheck with leak-check full: zero errors and zero leaks.
- Only iteration transform/packing/index buffers are retained; per-frame scan validation/extraction/covariance and PCL internal query allocations remain.
