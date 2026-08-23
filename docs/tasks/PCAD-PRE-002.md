# PCAD-PRE-002: deterministic voxel sampling and source mapping

## Identity

- Task ID: PCAD-PRE-002
- Milestone: M3 / B09
- Objective: Downsample valid millimetre points while retaining exact source membership.
- Dependencies: PCAD-PRE-001, PCAD-BE-001

## Allowed scope

- Files/modules that may change: private preprocessing contracts/implementation, tests, build files,
  and M3 documentation.
- Public API changes: none.
- New dependencies: none.

## Contract

- Inputs and ownership: borrow a masked, normalized `SurfaceView` and positive voxel size in mm.
- Outputs and ownership: own an unorganized centroid surface plus CSR offsets/source storage indices.
- Units and coordinate frames: input/output are millimetres in the unchanged input frame.
- Determinism requirements: floor-based integer keys, lexicographic voxel order, stable row-major source
  membership, double accumulation, and no randomized behavior.
- Error cases: invalid size/unit, coordinate-to-key overflow, or no valid input returns `Result`.
- Performance budget: O(n log v) time and O(n + v) memory for n inputs and v occupied voxels.

## Exclusions

- Explicitly out of scope: approximate/hash ordering, adaptive voxels, normal estimation, boundaries,
  GPU code, and topology reconstruction.
- Architecture boundaries that must not move: implementation is backend-neutral and does not expose
  PCL or Eigen.

## Acceptance

- Required unit tests: centroids, negative keys, invalid filtering, organized padding, CSR mapping,
  normal aggregation, metadata, invalid inputs, all-masked input, and repeatability.
- Required integration/golden tests: existing I/O, public-header, and consumer gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, architecture, changelog, and README.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`, and the Release
  `format`/`format-check` targets.
- Test results: Debug and Release each pass all 14 CTest tests; signed voxel ordering, centroids,
  CSR mapping, normals, padding/mask filtering, overflow/errors, repeatability, install, and consumer
  pass.
- Remaining risks: ordered-map implementation prioritizes reproducibility over peak throughput.
