# PCAD-REL-002: SDK release verification and evidence-based acceptance report

## Identity
- Task ID: PCAD-REL-002
- Milestone: M16
- Objective: SDK release verification and evidence-based acceptance report.
- Dependencies: PCAD-VALID-001.

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
- Windows CPU Debug/Release: 36/36 each; CUDA Debug/Release: 38/38 each. Both Release presets passed `-VerifyInstall -VerifyPackage`, including standalone consumers and relocated package runtime checks.
- Linux GCC Release: 36/36, installed and relocated consumers, TGZ package and CLI without `LD_LIBRARY_PATH`; target-scoped relative RPATH fixes relocation. Clang ASan/UBSan: 36/36.
- Generated acceptance: 200 core runs plus 32 relocated-package runs; format-check and configured static analysis passed with reviewed advisory diagnostics. GPU sanitizer results remain clean.
- Reproduction commands, raw package summaries, compatibility and measured scope: [release report](../release/M16.md).
- Remaining risks: labelled industrial scans, more physical hardware and deployment latency require separate site acceptance. Generated fixtures are the user-authorized validation scope.
