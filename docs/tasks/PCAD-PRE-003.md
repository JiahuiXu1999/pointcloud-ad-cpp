# PCAD-PRE-003: normal estimation, orientation, and boundary detection

## Identity

- Task ID: PCAD-PRE-003
- Milestone: M3 / B10
- Objective: Produce normal availability, orientation evidence, and boundary flags independently.
- Dependencies: PCAD-PRE-002, PCAD-BE-001

## Allowed scope

- Files/modules that may change: private preprocessing orchestration and PCL feature facade, tests,
  PCL CMake components, and M3 documentation.
- Public API changes: none.
- New dependencies: none.

## Contract

- Inputs and ownership: borrow a masked millimetre surface and validated radii/minimum-neighbor and
  optional orientation-hint values.
- Outputs and ownership: own a surface with unit normals, normal reasons, per-point orientation proof,
  boundary flags, and aggregate orientation status.
- Units and coordinate frames: radii and viewpoint positions are millimetres in the surface frame;
  direction hints are unitless vectors in that frame.
- Determinism requirements: stable input mapping, sorted radius-neighbor indices, fixed PCA and
  orientation rules, and deterministic angular-gap/grid boundary decisions.
- Error cases: invalid configuration/unit and backend failures return `Result`; insufficient local
  support marks `normal_missing` rather than throwing.
- Performance budget: radius searches dominate; storage and outputs remain linear in input size.

## Exclusions

- Explicitly out of scope: learned normals, global sign inference without evidence, defect labels,
  registration, and configurable angular boundary thresholds.
- Architecture boundaries that must not move: PCL/Eigen includes remain in `src/backends/pcl/`.

## Acceptance

- Required unit tests: planar PCA, minimum neighbors, existing-normal normalization, direction and
  viewpoint orientation, unproven orientation, grid depth boundaries, invalid input, and repeatability.
- Required integration/golden tests: AC-008/AC-009 foundations plus existing repository gates.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, architecture, changelog, README, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`, and the Release
  `format`/`format-check` targets.
- Test results: Debug and Release each pass all 15 CTest tests; PCA support, existing normals,
  direction/viewpoint orientation, unproven status, depth/radius boundaries, repeatability, install,
  and consumer pass.
- Remaining risks: industrial orientation hints and real depth-jump thresholds await the sample set
  tracked in `docs/validation/M3_INDUSTRIAL_SAMPLES.md`.
