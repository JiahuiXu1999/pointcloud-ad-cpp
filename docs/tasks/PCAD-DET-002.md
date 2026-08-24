# PCAD-DET-002: missing-material clustering

## Identity

- Task ID: PCAD-DET-002
- Milestone: M6 / B20
- Objective: Cluster reference points whose coverage reason is `no_neighbor` into missing-material
  regions, without region measurement or severity rules.
- Dependencies: PCAD-DET-001, PCAD-CMP-002

## Allowed scope

- Files/modules that may change: `src/detection/defect_cluster.{hpp,cpp}`, the existing PCL
  Euclidean clustering backend, the shared unit test, build files, and M6 documentation.
- Public API changes: none. `DefectType` gains the internal `missing_material` enumerator.
- New dependencies: none.

## Contract

- Inputs and ownership: `cluster_missing_material` borrows a millimetre reference, a
  `CoverageField`, and a validated `ValidatedDetectionConfig`.
- Outputs and ownership: `std::vector<DefectCluster>` typed `missing_material`, each owning member
  storage indices into the reference.
- Units and coordinate frames: cluster tolerance in millimetres.
- Determinism requirements: candidate collection and label assignment follow reference storage
  order.
- Error cases: non-millimetre reference and a coverage field that does not span the reference
  storage return `Result` failures.
- Performance budget: one radius-search clustering pass over the uncovered reference points.

## Exclusions

- Explicitly out of scope: region measurement and severity rules (PCAD-DET-003), and any verdict
  coupling.
- Architecture boundaries that must not move: PCL stays behind `src/backends/pcl/`.

## Acceptance

- Required unit tests: AC-005 asserts a single missing block forms one `missing_material` cluster
  whose members are all `no_neighbor` reference points.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the extended defect-cluster test passes alongside the full suite.
- Remaining risks: clusters are consumed by PCAD-DET-003 for measurement and severity.
