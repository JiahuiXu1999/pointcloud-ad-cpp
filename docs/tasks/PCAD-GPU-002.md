# PCAD-GPU-002: Exact GPU KNN, covariance preparation and resident GICP objectives

## Identity
- Task ID: PCAD-GPU-002
- Milestone: M14
- Objective: Exact GPU KNN, covariance preparation and resident GICP objectives.
- Dependencies: PCAD-GPU-001.

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

- Completed 2026-09-27: CPU Debug/Release 35/35, CUDA Debug/Release 37/37; format-check and configured backend/test static analysis passed (style advisories retained). Final CUDA paths passed memcheck with full leak checking, racecheck and synccheck with zero errors/leaks/hazards.
- Exact KNN brute-force oracle covers ties, duplicates and K=1/3/12/32/all points. Near-first bounded-stack traversal preserves exact `(distance,index)` results. Bounded 16k-query batches avoid N*K workspace growth.
- Device covariance matches independent Eigen host eigensolutions within 1e-10; nonzero rotated GICP equations match scalar information-matrix arithmetic within 1e-9 relative scale. Existing heterogeneous pose tolerances and frozen scalar regression remain unchanged and pass.
- GPU reference covariance remains resident; per-frame scan index/covariance is rebuilt once, then GICP rotations/information/objective/final metrics stay on-device. Degenerate scan rejection followed by successful reuse is tested.
- Documented the formerly unspecified covariance cutoff tie rule in the numerical contract. Cross-backend bitwise identity is not claimed.
