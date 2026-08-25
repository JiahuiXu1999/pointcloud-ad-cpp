# PCAD-SER-001: config/result schemas and JSON

## Identity

- Task ID: PCAD-SER-001
- Milestone: M7 / B23
- Objective: Implement versioned JSON serialization for the inspection configuration and result,
  behind an internal serialization facade.
- Dependencies: PCAD-PIPE-001

## Allowed scope

- Files/modules that may change: `src/serialization/json_serialization.{hpp,cpp}`, its unit test,
  build files, the vcpkg manifest, and the nlohmann-json ADR.
- Public API changes: none. The serialization facade is internal and exchanges only project-owned
  value types.
- New dependencies: nlohmann-json (ADR-0008).

## Contract

- Inputs and ownership: `serialize_config`/`parse_config` exchange `InspectionConfig`;
  `serialize_result` takes `InspectionResult` and returns an owned JSON string.
- Outputs and ownership: versioned JSON documents with lowercase snake_case enums, UTC RFC 3339
  timestamps, integer microseconds, and null for unavailable or non-finite measurements.
- Units and coordinate frames: unchanged; transforms serialize as a 16-element row-major matrix with
  source/target frames.
- Determinism requirements: stable key ordering and number formatting.
- Error cases: an unknown schema major version, malformed JSON, and invalid enum values return
  `Result` failures.
- Performance budget: linear in document size.

## Exclusions

- Explicitly out of scope: PLY fields, manifest, and hashes (PCAD-SER-002), and the CLI
  (PCAD-CLI-001).
- Architecture boundaries that must not move: nlohmann/json never appears in installed headers.

## Acceptance

- Required unit tests: config round-trip, lowercase enum serialization, and null for non-finite
  numbers.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, ADR, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the new serialization test passes alongside the full suite.
- Remaining risks: result JSON golden files arrive with the M8 acceptance matrix.
