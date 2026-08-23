# PCAD-GEO-001: borrowed and owned surface contracts

## Identity

- Task ID: PCAD-GEO-001
- Milestone: M2 / B04
- Objective: Introduce validated backend-neutral borrowed and owned surface representations.
- Dependencies: PCAD-CORE-002

## Allowed scope

- Files/modules that may change: public surface header, dependency-free ownership/validation tests,
  CMake registration, and M2 documentation.
- Public API changes: add `GridTopology`, `SurfaceView`, and `OwnedSurface`.
- New dependencies: none.

## Contract

- Inputs and ownership: `SurfaceView` borrows arrays and frame metadata; `OwnedSurface` moves and
  owns immutable vectors and its frame.
- Outputs and ownership: `OwnedSurface::view()` borrows from the owner and is valid only while the
  owner remains alive and unmoved.
- Units and coordinate frames: surfaces retain their declared unit and case-sensitive frame; no
  conversion occurs in this task.
- Determinism requirements: validation walks points in stable row-major order and reports the first
  failing contract.
- Error cases: empty points, length mismatches, invalid masks, unmasked non-finite values, and
  inconsistent topology return `invalid_input`.
- Performance budget: validation is linear; `view()` is allocation-free.

## Exclusions

- Explicitly out of scope: unit/frame normalization, PCL conversion, and file I/O.
- Architecture boundaries that must not move: public types expose only project/standard types and
  never expose mutable owning storage.

## Acceptance

- Required unit tests: ownership traits, empty/mismatched arrays, masked/unmasked non-finite values,
  mask domain, organized topology, row padding, and metadata preservation.
- Required integration/golden tests: existing install/consumer tests remain green.
- Required build presets: `windows-msvc-debug`; M2 completion also verifies Release install.
- Required documentation updates: roadmap, README, architecture, and changelog.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`, and
  `cmake --build --preset windows-msvc-debug --target format-check`.
- Test results: Debug 9/9 passed; Release 9/9 passed; install/export verification and the external
  consumer build/run passed using the installed `surface.hpp` header.
- Remaining risks: a standalone `SurfaceView` borrows its `FrameId`; callers must preserve the frame
  for the same lifetime as the borrowed arrays.
