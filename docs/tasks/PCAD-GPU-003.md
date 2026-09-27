# PCAD-GPU-003: Resident transfer accounting, sanitizer and scale acceptance

## Identity
- Task ID: PCAD-GPU-003
- Milestone: M14
- Objective: Resident transfer accounting, sanitizer and scale acceptance.
- Dependencies: PCAD-GPU-002.

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

- Completed 2026-09-27: CPU Debug/Release 35/35, CUDA Debug/Release 37/37, format-check and configured benchmark static analysis passed. GPU memcheck with full leak checking, racecheck and synccheck reported zero errors/leaks/hazards.
- Transfer counters and tests verify eight evaluations transfer exactly 8*360 bytes and do not upload point arrays. Payload accounting includes GICP scan resources and bounded KNN scratch.
- Sequential Release CPU/GPU benchmark at 4,225 / 100,489 / 1,000,000 points passed all accuracy/parity/reuse gates. See [report and raw data](../benchmarks/PCAD-GPU-003.md); CPU exact-cutoff overhead is reported explicitly.
