# PCAD-VALID-001: Cross-platform/backend regression and generated ground-truth acceptance

## Identity
- Task ID: PCAD-VALID-001
- Milestone: M16
- Objective: Cross-platform/backend regression and generated ground-truth acceptance.
- Dependencies: PCAD-POLICY-001.

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
- Windows CPU Debug/Release 36/36; CUDA Debug/Release 38/38. Generated file acceptance: CPU 32, CUDA 48, large CUDA 24, AVX2-disabled CPU 32 cases passed.
- AVX2-off/on full suites passed; GPU memcheck/leaks/racecheck/synccheck clean. Windows format and configured static analysis completed.
- [Dataset, truth, raw summaries and validation protocol](../validation/PCAD-VALID-001.md).
- Linux GCC 11.4 Release: 36/36, 32 generated cases and installed consumer; Clang 21.1.8 ASan+UBSan: 36/36 and 32 generated cases. Cross-platform report/count/fixture-hash gates passed. Total generated runs: 200.
- Initial Clang 14 startup failures were reproduced with an empty program and resolved using the upstream-fixed runtime; no sanitizer/ASLR settings were weakened.
- Remaining risks: WSL shares the same physical CPU; industrial samples and other physical hardware remain deployment gates, outside the user-authorized generated-data acceptance.

- User clarification (2026-09-27): no industrial dataset is currently available; generate data for validation. Implement fixed-seed normal/dent/bump/missing and robustness fixtures with explicit truth and expected verdict/metrics. This closes the requested synthetic acceptance scope; real industrial/site acceptance and additional physical devices remain separately labelled deployment gates.
