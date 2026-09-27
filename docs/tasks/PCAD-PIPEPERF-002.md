# PCAD-PIPEPERF-002: Bounded parallel normal and boundary preprocessing

## Identity
- Task ID: PCAD-PIPEPERF-002
- Milestone: M15
- Objective: Bounded parallel normal and boundary preprocessing.
- Dependencies: PCAD-PIPEPERF-001.

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
- Windows CPU Debug/Release: 36/36 each; CUDA Debug/Release: 38/38 each. Format check passed. Configured clang-tidy completed; reviewed existing shared_ptr-return ownership false positive in make_valid_cloud (RAII owns the allocation) and adjacent numeric parameter advisories; no suppressions or check changes.
- Fixed 256-point blocks, bounded workers and worker-local search buffers preserve exact normals, validity/orientation and boundaries for 1/2/4/8 workers. Zero thread budgets are rejected.
- Full inspection measurements: [raw CSV](../benchmarks/PCAD-PIPEPERF-002.csv), 100489 points, four workers, CPU and GPU registration.
- Remaining risks: timings describe this host; PCL trees are immutable while independent query buffers are used. Industrial accuracy remains separate from generated-data acceptance.
