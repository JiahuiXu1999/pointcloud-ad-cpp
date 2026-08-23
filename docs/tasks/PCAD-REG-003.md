# PCAD-REG-003: registration quality gate

## Identity

- Task ID: PCAD-REG-003
- Milestone: M4 / B13
- Objective: Implement the backend-neutral registration quality gate that decides whether a solve's
  metrics are trustworthy, without coupling to verdicts or process exit codes.
- Dependencies: PCAD-REG-002, PCAD-REG-001

## Allowed scope

- Files/modules that may change: `src/registration/registration_gate.{hpp,cpp}`, its unit test,
  build files, and M4 documentation.
- Public API changes: none. The gate is an internal `pointcloud_ad::registration` module consuming
  the public `RegistrationMetrics` and `ValidatedRegistrationGateConfig` types.
- New dependencies: none.

## Contract

- Inputs and ownership: `evaluate_gate` borrows `RegistrationMetrics` and
  `ValidatedRegistrationGateConfig`; neither is retained beyond the call.
- Outputs and ownership: `RegistrationGateResult` carries a `passed` flag, a first-failure
  `GateFailure` reason, and a human-readable message.
- Determinism requirements: gates are evaluated in a fixed specification order (convergence, pair
  count, overlap, residual, translation prior, rotation prior) so the first failure is stable.
- Error cases: an internal exception during evaluation maps to an `internal_error` `Result`; a
  rejected solve is a successful `Result` whose value has `passed == false`.
- Performance budget: constant-time field comparisons.

## Exclusions

- Explicitly out of scope: forming a verdict (PASS/FAIL/INDETERMINATE), mapping to exit codes, and
  any defect-detection or pipeline orchestration.
- Architecture boundaries that must not move: the gate never includes PCL or Eigen and never decides
  a business verdict, preserving the PCAD-REG-001 contract separation.

## Acceptance

- Required unit tests: each gate rejects its own failing condition (convergence, pairs, overlap,
  residual, translation, rotation) and a boundary solve exactly at every threshold passes.
- Required integration/golden tests: the ICP test's grossly-wrong-initial-pose case (AC-007) is
  rejected by this gate; existing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the gate unit test and the ICP/gate integration case pass alongside the full suite.
- Remaining risks: the gate result is consumed by the M7 pipeline to form `INDETERMINATE` reports.
