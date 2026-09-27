# PCAD-PIPEPERF-003: Parallel comparison, measured voxel/clustering policy and complete inspection benchmark

## Identity
- Task ID: PCAD-PIPEPERF-003
- Milestone: M15
- Objective: Parallel comparison, measured voxel/clustering policy and complete inspection benchmark.
- Dependencies: PCAD-PIPEPERF-002.

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
- CPU Debug/Release 36/36; CUDA Debug/Release 38/38. Format and configured static analysis completed.
- Exact threaded comparison tests include padded grids, masks, duplicates/ties and inclusive radius. Complete inspection benchmark ran 4225/100489/1000000 points at 1/4 workers, CPU/GPU, fresh/cached; defect workload also passed.
- [Report, raw evidence and measured voxel/clustering policy](../benchmarks/PCAD-PIPEPERF-003.md).
- Remaining risks: reference comparison trees are rebuilt; defect-heavy clustering needs separate profiling; generated fixtures do not establish production accuracy.
