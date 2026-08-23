# PCAD-CORE-001: public error and result contracts

## Identity

- Task ID: PCAD-CORE-001
- Milestone: M1 / B01
- Objective: Introduce dependency-free program error and move-only value-or-error contracts.
- Dependencies: PCAD-ENG-003

## Allowed scope

- Files/modules that may change: public core headers, dependency-free unit tests, CMake header and
  test registration, and M1 documentation.
- Public API changes: add `ErrorCode`, `PipelineStage`, `Error`, and `Result<T>`.
- New dependencies: none.

## Contract

- Inputs and ownership: `Error` owns its message and ordered context; `Result<T>` owns exactly one
  value or error, is move-constructible, and deliberately disables assignment so a throwing held
  type cannot make the carrier valueless.
- Outputs and ownership: accessors return references to owned state; rvalue access transfers state.
- Units and coordinate frames: not applicable.
- Determinism requirements: error context uses ordered keys.
- Error cases: inactive `value()`/`error()` access is a documented precondition violation; no
  exception-based access API is exposed.
- Performance budget: one discriminated value with no heap allocation beyond the held types.

## Exclusions

- Explicitly out of scope: inspection verdicts, JSON serialization, logging, and algorithms.
- Architecture boundaries that must not move: core remains independent of I/O, CLI, JSON, and PCL.

## Acceptance

- Required unit tests: success, failure, ordered context, `value_or`, and move-only value transport.
- Required integration/golden tests: existing DLL and consumer gates remain green.
- Required build presets: `windows-msvc-debug`; M1 completion also verifies Release install.
- Required documentation updates: roadmap, README, architecture summary, and changelog.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`, and the Release
  `format-check` target.
- Test results: Debug and Release pass all 7 CTest tests; Release installation, DLL export/runtime
  audit, and independent installed consumer pass.
- Remaining risks: inactive-alternative access is a precondition, matching the deliberately small
  v0.1 API rather than providing exception-throwing checked access.
