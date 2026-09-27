# PCAD-PIPEPERF-001: Owned reusable inspection reference and registration context

## Identity
- Task ID: PCAD-PIPEPERF-001
- Milestone: M15
- Objective: Owned reusable inspection reference and registration context.
- Dependencies: PCAD-GPU-003.

## Allowed scope
- Files/modules: corresponding private backends, pipeline/registration adapters, public facade where needed, tests, benchmarks, scripts and documentation.
- Public API changes: small typed ownership/execution controls; no device or third-party types.
- New dependencies: none.

## Contract
- Inputs borrowed synchronously; reusable contexts own reference snapshots and mutable work resources.
- Outputs own their data; millimetres, right-handed scan-to-reference transforms.
- Preserve masks, logical storage mapping, tie ordering, thresholds and failure/verdict semantics.
- Repeated same-backend calls are deterministic; fixed block ordering; cross-device arithmetic uses existing metric tolerances, not bitwise identity.
- Failure returns Result across the public API; drain work before returning; contexts remain reusable after rejected input.
- Performance: record complete calls, preparation/reuse, transfer volume and memory where relevant; no invented speedups.

## Exclusions
- No new registration/detection semantics, approximate search, public PCL/CUDA/Eigen types or process-wide execution policy.
- No production or industrial acceptance claims without actual evidence.

## Acceptance
- Unit/contract tests before or with changes, including boundaries, masks, degeneracy, determinism and lifetime failures.
- Existing integration/acceptance suite; CPU/CUDA parity where applicable.
- Windows CPU Debug/Release; CUDA Debug/Release for GPU/shared solver changes; Release installed consumer for public or install changes.
- Format-check and configured static analysis; GPU sanitizers for device/resource changes.
- Reproducible report and roadmap/task-card update. External platform/industrial gates remain open until executed.

## Completion evidence
- Status: Complete.
- Commands/results: recorded as executed.
- Remaining risks: external hardware and labelled industrial data may require user-provided resources.

- Completed 2026-09-27: CPU Debug/Release 36/36; CUDA Debug/Release 38/38; both Release installed consumers passed with the new exported InspectionContext API. Format and configured pipeline/test/benchmark static analysis passed (style advisories retained).
- Tests compare fresh/cached registration, deviation/coverage and all region measurements on normal and changing defect scans; reference mutation/lifetime, moved-from errors, capacity, wrong frame, independent concurrency, organized row padding and masks are covered. GPU context memcheck reports zero errors/leaks.
- Release 100,489-point full-inspection fresh/prepare/first/reused baseline with CPU/GPU and four workers: [raw data](../benchmarks/PCAD-PIPEPERF-001.csv). Correctness gates pass; reference preparation is excluded from per-frame timings and documented in SDK ownership contracts.
