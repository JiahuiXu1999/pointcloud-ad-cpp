# PCAD-POLICY-001: Measured backend selection with explicit overrides and observable decisions

## Identity
- Task ID: PCAD-POLICY-001
- Milestone: M16
- Objective: Measured backend selection with explicit overrides and observable decisions.
- Dependencies: PCAD-PIPEPERF-003.

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
- CPU Debug/Release 36/36 and CUDA Debug/Release 38/38; both Release installed consumers passed, including the exported backend observer. Format check passed; configured static analysis completed (moved-from test calls intentionally exercise the documented rejection/sentinel contract).
- Pure policy threshold/availability/override tests plus a real 65536-point automatic context and fresh/cached inspection verify geometry and actual-backend reporting.
- SDK documents immutable capacity-based selection, explicit overrides, error behavior, cold-start and hardware limitations, and the updated GPU execution-scope string.
- Remaining risks: one-host warm-registration threshold is conservative, not an all-hardware/full-pipeline speed guarantee; allocation/runtime failures remain explicit errors.
