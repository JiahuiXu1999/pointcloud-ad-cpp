# PCAD-DET-001: dent/bump classification and clustering

## Identity

- Task ID: PCAD-DET-001
- Milestone: M6 / B19
- Objective: Classify trustworthy deviation samples into dent and bump candidates and cluster each
  class with 3D Euclidean clustering, without missing-material detection or region measurement.
- Dependencies: PCAD-CMP-003, PCAD-BE-001

## Allowed scope

- Files/modules that may change: `src/detection/defect_cluster.{hpp,cpp}`, the PCL Euclidean
  clustering backend `src/backends/pcl/pcl_cluster_backend.{hpp,cpp}`, their unit test, build
  files, and M6 documentation.
- Public API changes: none. Detection is an internal `pointcloud_ad::detection` module.
- New dependencies: none. Clustering is implemented with a union-find over PCL radius search.

## Contract

- Inputs and ownership: `cluster_deviation_defects` borrows an aligned millimetre scan, a
  `DeviationField`, and a validated `ValidatedDetectionConfig`.
- Outputs and ownership: `std::vector<DefectCluster>`, each owning its member storage indices and a
  `DefectType` (dent or bump).
- Units and coordinate frames: thresholds and cluster tolerance in millimetres.
- Determinism requirements: candidate collection and cluster-label assignment follow storage order.
- Error cases: non-millimetre scan and a deviation field that does not span the scan storage return
  `Result` failures.
- Performance budget: two radius-search clustering passes, one per defect class.

## Exclusions

- Explicitly out of scope: missing-material clustering (PCAD-DET-002), region measurement and
  severity rules (PCAD-DET-003), and any verdict coupling.
- Architecture boundaries that must not move: PCL stays behind `src/backends/pcl/`.

## Acceptance

- Required unit tests: AC-003/004 assert a single bump/dent forms exactly one correctly typed
  cluster whose members all exceed the threshold, a clean surface forms no cluster, and a bump below
  the minimum point count is filtered out.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the new defect-cluster test passes alongside the full suite.
- Remaining risks: clusters are consumed by PCAD-DET-003 for measurement and severity.
