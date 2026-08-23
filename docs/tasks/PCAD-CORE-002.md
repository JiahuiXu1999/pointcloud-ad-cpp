# PCAD-CORE-002: units, frames, vectors, and rigid transforms

## Identity

- Task ID: PCAD-CORE-002
- Milestone: M1 / B02
- Objective: Add dependency-free geometry value contracts with explicit units and frame direction.
- Dependencies: PCAD-CORE-001

## Allowed scope

- Files/modules that may change: public geometry header, dependency-free transform tests, CMake
  header/test registration, and M1 documentation.
- Public API changes: add `LengthUnit`, `Vec3f`, `Vec3d`, `FrameId`, unit conversion, and
  `RigidTransform`.
- New dependencies: none.

## Contract

- Inputs and ownership: frame identifiers and transform matrices are copied or moved into owning
  values; frame IDs must be non-empty valid UTF-8.
- Outputs and ownership: transform accessors borrow immutable state; transformed points are values.
- Units and coordinate frames: translations and transformed points are millimetres; transforms are
  right-handed, stored row-major, applied to column vectors, and directed source to target.
- Determinism requirements: conversion and matrix validation contain no randomized behavior.
- Error cases: non-finite lengths/matrices, unknown units, invalid frames, non-homogeneous matrices,
  non-orthonormal rotations, and reflections return `invalid_input`.
- Performance budget: point application performs a fixed affine multiply with no allocation.

## Exclusions

- Explicitly out of scope: surface ownership, point-cloud normalization, and PCL/Eigen integration.
- Architecture boundaries that must not move: installed headers contain only project and standard
  library types.

## Acceptance

- Required unit tests: unit scale, invalid unit/value, UTF-8 frame validation, transform direction,
  column-vector application, scale rejection, and reflection rejection.
- Required integration/golden tests: existing DLL and consumer gates remain green.
- Required build presets: `windows-msvc-debug`; M1 completion also verifies Release install.
- Required documentation updates: roadmap, README, architecture summary, and changelog.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`, and the Release
  `format-check` target.
- Test results: Debug and Release pass all 7 CTest tests, including transform and public-header
  boundary tests; installed consumer compiles the M1 headers.
- Remaining risks: tolerance is caller-adjustable for imported transforms; the default is strict for
  programmatically constructed matrices and later adapters must select an input-appropriate value.
