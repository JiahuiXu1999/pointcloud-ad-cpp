# PCAD-IO-001: PLY reader/writer adapter

## Identity

- Task ID: PCAD-IO-001
- Milestone: M2 / B07
- Objective: Read and write deterministic PLY surfaces through the private PCL backend.
- Dependencies: PCAD-BE-001, ADR-0002

## Allowed scope

- Files/modules that may change: private I/O contract, PCL I/O backend, round-trip tests, build files,
  and M2 documentation.
- Public API changes: none.
- New dependencies: none; use the pinned PCL I/O component.

## Contract

- Inputs and ownership: UTF-8 path text and surface views are borrowed for each call.
- Outputs and ownership: reads return an immutable `OwnedSurface`; writes return the logical point
  count after the file has been closed successfully.
- Units and coordinate frames: PLY does not define project semantics, so the caller supplies unit and
  frame on read; writing performs no conversion.
- Determinism requirements: preserve logical row-major order, explicit masks, optional normals, and
  organized dimensions; binary little-endian is the default encoding.
- Error cases: empty/wrong-extension paths, missing or malformed XYZ/normal/mask fields, corrupt
  files, and backend failures return stable `Result` errors.
- Performance budget: one linear project/backend conversion plus PCL file I/O.

## Exclusions

- Explicitly out of scope: PCD, meshes/faces, colors, arbitrary attributes, unit inference, public
  file APIs, serialization artifacts, and CLI commands.
- Architecture boundaries that must not move: only `src/backends/pcl/` includes PCL; `src/io/` uses
  project-owned values and a private facade.

## Acceptance

- Required unit tests: binary and ASCII round trips, points, normals, masks, organized topology,
  semantic metadata, missing XYZ, corrupt input, and path validation.
- Required integration/golden tests: public include audit and installed consumer remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, architecture, changelog, and README.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`, and the Release
  `format`/`format-check` targets.
- Test results: Debug and Release each pass all 11 CTest tests; binary/ASCII PLY round trips,
  finite-value mask derivation, format errors, public-header audit, install, and consumer pass.
- Remaining risks: PLY cannot carry standardized unit/frame semantics; callers must supply them.
