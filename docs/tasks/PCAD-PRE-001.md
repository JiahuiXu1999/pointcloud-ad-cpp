# PCAD-PRE-001: finite-value, ROI, and valid-mask stage

## Identity

- Task ID: PCAD-PRE-001
- Milestone: M3 / B08
- Objective: Produce a deterministic preprocessing mask with one traceable reason per input slot.
- Dependencies: PCAD-GEO-002

## Allowed scope

- Files/modules that may change: private preprocessing contracts/implementation, unit tests, build
  files, and M3 documentation.
- Public API changes: none.
- New dependencies: none.

## Contract

- Inputs and ownership: borrow a normalized millimetre `SurfaceView` and optional inclusive AABB ROI.
- Outputs and ownership: own an unchanged surface copy, explicit valid mask, per-storage-slot reason,
  and logical valid count.
- Units and coordinate frames: input and ROI coordinates are millimetres in the surface frame.
- Determinism requirements: stable row-major traversal and fixed reason precedence.
- Error cases: non-millimetre input and invalid/non-finite ROI bounds return `Result` errors.
- Performance budget: one linear pass and one owned copy of each input array.

## Exclusions

- Explicitly out of scope: voxel sampling, normals, boundary detection, named ROI labels, and PCL.
- Architecture boundaries that must not move: preprocessing depends only on domain geometry.

## Acceptance

- Required unit tests: finite/non-finite values, input masks, inclusive ROI, normals, organized row
  padding, invalid units/ROI, reason precedence, and repeatability.
- Required integration/golden tests: existing I/O, public-header, and consumer gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, architecture, changelog, and README.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`, and the Release
  `format`/`format-check` targets.
- Test results: Debug and Release each pass all 13 CTest tests; mask/reason precedence, inclusive
  ROI, row padding, unit/ROI errors, repeatability, I/O regressions, install, and consumer pass.
- Remaining risks: named/multiple ROI policies remain a later product configuration concern.
