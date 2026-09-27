# PCAD-GPU-001: Resident scan, GPU point-to-plane/point-to-point objectives and compact deterministic reduction

## Identity
- Task ID: PCAD-GPU-001
- Milestone: M14
- Objective: Resident scan, GPU point-to-plane/point-to-point objectives and compact deterministic reduction.
- Dependencies: PCAD-SIMD-003.

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

- Completed 2026-09-27: `scripts/build.ps1` CPU Debug/Release 35/35, CUDA Debug/Release 37/37. `format-check` passed; configured clang-tidy on the shared backend completed (advisory style diagnostics retained).
- Compute Sanitizer memcheck/racecheck/synccheck on `pointcloud_ad_cuda_resident_tests`: zero errors and race hazards. Existing heterogeneous and context tests retain all parity/lifetime checks.
- Added scalar-equation oracle, fixed reduction repeatability, 1/127/128/129/257 point tails, no-neighbor, back-face, invalid transformed coordinate and recovery tests. Each iteration transfers only 360 bytes of equation/statistics back to the host.
- No public API or dependencies changed. Scale/transfer measurements continue in GPU-003 after the GICP path is complete.
