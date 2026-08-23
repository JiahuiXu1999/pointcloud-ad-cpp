# PCAD-IO-002: PCD reader/writer adapter

## Identity

- Task ID: PCAD-IO-002
- Milestone: M2 / B07
- Objective: Read and write deterministic PCD surfaces through the private PCL backend.
- Dependencies: PCAD-BE-001, PCAD-IO-001, ADR-0002

## Allowed scope

- Files/modules that may change: private I/O contract, shared PCL field codec, PCD adapter and tests,
  build files, and M2 documentation.
- Public API changes: none.
- New dependencies: none; use the pinned PCL I/O component.

## Contract

- Inputs and ownership: UTF-8 path text and surface views are borrowed for each call.
- Outputs and ownership: reads return an immutable `OwnedSurface`; writes return the logical count.
- Units and coordinate frames: caller-supplied on read and never inferred from PCD metadata.
- Determinism requirements: stable row-major order, masks, optional normals, and organized topology.
- Error cases: invalid paths, missing/malformed fields, corrupt input, and backend failures return
  stable `Result` errors.
- Performance budget: one linear conversion plus PCL file I/O.

## Exclusions

- Explicitly out of scope: compressed PCD writes, arbitrary attributes, CLI/public I/O, and M3.
- Architecture boundaries that must not move: PCL includes remain beneath `src/backends/pcl/`.

## Acceptance

- Required unit tests: binary/ASCII round trips, masks, normals, topology, metadata, corrupt and
  missing-field inputs, and path validation.
- Required integration/golden tests: public include audit and installed consumer remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, architecture, changelog, and README.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`, and the Release
  `format`/`format-check` targets.
- Test results: Debug and Release each pass all 12 CTest tests; binary/ASCII PCD round trips,
  finite-value mask derivation, field/corruption errors, PLY regression, install, and consumer pass.
- Remaining risks: caller-supplied unit/frame semantics cannot be recovered from generic PCD files.
