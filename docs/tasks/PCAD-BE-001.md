# PCAD-BE-001: internal PCL facade and surface conversions

## Identity

- Task ID: PCAD-BE-001
- Milestone: M2 / B06
- Objective: Isolate PCL behind a project-owned internal facade and deterministic conversions.
- Dependencies: PCAD-GEO-001, ADR-0002

## Allowed scope

- Files/modules that may change: PCL backend, private backend tests, vcpkg/CMake integration, and
  M2 documentation.
- Public API changes: none.
- New dependencies: PCL 1.15.1#1 through the pinned vcpkg manifest approved by ADR-0002.

## Contract

- Inputs and ownership: conversion borrows a validated `SurfaceView`; `PclSurface` uniquely owns
  opaque backend state.
- Outputs and ownership: reverse conversion returns an immutable `OwnedSurface`.
- Units and coordinate frames: the backend carries no semantic units/frame; reverse conversion
  requires both explicitly and performs no scaling.
- Determinism requirements: logical points are copied in stable row-major order.
- Error cases: moved-from/inconsistent backend state and caught backend exceptions return `Result`.
- Performance budget: one linear copy per direction; organized row padding is compacted once.

## Exclusions

- Explicitly out of scope: file paths, PLY/PCD parsing, normal estimation, filtering, and public API.
- Architecture boundaries that must not move: PCL includes remain under `src/backends/pcl/`; I/O and
  installed headers use only project-owned types.

## Acceptance

- Required unit tests: points, normals, masks, organized topology, padded rows, and semantic metadata.
- Required integration/golden tests: public include audit and installed consumer remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, architecture, changelog, and dependency ADR.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug -Fresh` and
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -Fresh -VerifyInstall`.
- Test results: Debug and Release each pass all 10 CTest tests; the PCL conversion test, public-header
  audit, one-symbol DLL export audit, installed CLI, and independent installed consumer all pass.
- Remaining risks: backend file adapters must derive masks when a file has no project sidecar mask.
