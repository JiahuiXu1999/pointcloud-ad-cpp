# PCAD-GEO-002: unit and frame normalization

## Identity

- Task ID: PCAD-GEO-002
- Milestone: M2 / B05
- Objective: Normalize surface coordinates once into millimetres and an explicit target frame.
- Dependencies: PCAD-GEO-001

## Allowed scope

- Files/modules that may change: public normalization header, dependency-free normalization tests,
  CMake registration, and M2 documentation.
- Public API changes: add `normalize_surface`.
- New dependencies: none.

## Contract

- Inputs and ownership: input surface data is borrowed for the call; the target frame and optional
  transform are values.
- Outputs and ownership: success owns a new immutable surface; input storage is never modified.
- Units and coordinate frames: points are converted once to millimetres before an optional matching
  source-to-target transform; normals are rotated only; output uses the target frame.
- Determinism requirements: row-major traversal and arithmetic order are fixed.
- Error cases: unsupported units, missing/misdirected transforms, non-representable valid output,
  and caught internal exceptions return `Result` errors.
- Performance budget: one allocation/copy per present array and one linear transform pass.

## Exclusions

- Explicitly out of scope: coordinate-frame graph lookup, normal estimation, and PCL conversion.
- Architecture boundaries that must not move: normalization remains backend- and I/O-independent.

## Acceptance

- Required unit tests: unit scaling, metadata, point transform direction, normal rotation, mask/grid
  preservation, missing transform, and wrong frame direction.
- Required integration/golden tests: existing install/consumer tests remain green.
- Required build presets: `windows-msvc-debug`; M2 completion also verifies Release install.
- Required documentation updates: roadmap, README, architecture, and changelog.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`, and
  `cmake --build --preset windows-msvc-debug --target format-check`.
- Test results: Debug 9/9 passed; Release 9/9 passed; install/export verification and the external
  consumer build/run passed using the installed `normalization.hpp` header.
- Remaining risks: transform discovery/composition is intentionally outside v0.1's normalization
  primitive; callers supply one proven direct transform.
