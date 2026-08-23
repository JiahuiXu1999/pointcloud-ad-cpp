# PCAD-CFG-001: strongly typed validated configuration

## Identity

- Task ID: PCAD-CFG-001
- Milestone: M1 / B03
- Objective: Define the v0.1 in-memory configuration contract and reject missing or unsafe values.
- Dependencies: PCAD-CORE-001 and PCAD-CORE-002

## Allowed scope

- Files/modules that may change: public configuration header, dependency-free validation tests,
  CMake header/test registration, and M1 documentation.
- Public API changes: add raw configuration groups, immutable validated groups,
  `ValidatedInspectionConfig`, and `validate_config`.
- New dependencies: none.

## Contract

- Inputs and ownership: validation takes an owning `InspectionConfig` value and moves validated
  strings/frames into an immutable validated object.
- Outputs and ownership: the result owns either the complete validated configuration or an error
  whose ordered context identifies the first invalid field.
- Units and coordinate frames: all algorithm thresholds are explicitly suffixed `_mm`, `_deg`, or
  `_rad`; input units and frames are strongly typed.
- Determinism requirements: deterministic mode, thread count, and random seed are required and
  retained for provenance.
- Error cases: every required field, finite/range/sign rule, production gate, coverage gate, and
  measurement-error-budget relation is validated before construction succeeds.
- Performance budget: one linear validation pass with no I/O or global state.

## Exclusions

- Explicitly out of scope: JSON parsing/schema files, production profile records, surface-dependent
  normal availability checks, and pipeline orchestration.
- Architecture boundaries that must not move: configuration remains independent of JSON, CLI, I/O,
  and PCL; no production threshold receives an implicit default.

## Acceptance

- Required unit tests: documented valid profile plus missing, range, non-finite, sign, budget,
  coverage/gate, thread, and seed failures with stable field context.
- Required integration/golden tests: existing DLL and installed consumer gates remain green.
- Required build presets: `windows-msvc-debug` and `windows-msvc-release -VerifyInstall`.
- Required documentation updates: roadmap, README, architecture summary, and changelog.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`, and the Release
  `format-check` target.
- Test results: Debug and Release pass all 7 CTest tests; the invalid-config matrix, installed M1
  headers, DLL export/runtime audit, and independent consumer pass.
- Remaining risks: profile-to-calibration/R&R document traceability is external metadata and belongs
  to the later versioned JSON schema task; M1 enforces the numeric error-budget relationship.
