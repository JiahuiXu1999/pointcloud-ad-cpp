# PCAD-SER-002: PLY fields, manifest, and hashes

## Identity

- Task ID: PCAD-SER-002
- Milestone: M7 / B24
- Objective: Implement the output layer for PLY fields, a run manifest, and content hashes behind an
  internal serialization facade, without changing status/exit-code semantics or exposing PCL/Eigen
  from installed headers.
- Dependencies: PCAD-SER-001

## Allowed scope

- Files/modules that may change: `src/serialization/output_writer.{hpp,cpp}`, its unit test, and
  build files.
- Public API changes: none. The facade is internal and exchanges only project-owned value types.
- New dependencies: none.

## Contract

- Inputs and ownership: `sha256_hex` takes a byte view and returns a lowercase hex digest;
  `write_attribute_ply` borrows a `SurfaceView` plus parallel scalar/reason/region spans and writes
  an owned ASCII PLY; `build_manifest` takes a schema version, a generator string, and a list of
  path/digest entries and returns an owned JSON document.
- Outputs and ownership: an ASCII PLY carrying `x`, `y`, `z`, `scalar`, `reason`, and `region_id`
  per vertex; a versioned manifest document listing every output file with its SHA-256 digest.
- Units and coordinate frames: millimetres; unchanged.
- Determinism requirements: fixed field order and lowercase hex digests; manifest key ordering is
  stable.
- Error cases: mismatched attribute spans, unopenable output paths, and write failures return
  `Result` failures.
- Performance budget: linear in surface size and document size.

## Exclusions

- Explicitly out of scope: exposing per-point fields through the pipeline public API, and the CLI
  (PCAD-CLI-001).
- Architecture boundaries that must not move: nlohmann/json never appears in installed headers; no
  PCL/Eigen dependency is introduced.

## Acceptance

- Required unit tests: SHA-256 against published test vectors, ASCII attribute PLY round-trip with
  the declared scalar/reason/region_id properties, span-size mismatch rejection, and manifest JSON
  structure with 64-character digests.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the new output-writer test passes alongside the full suite.
- Remaining risks: per-point PLY emission is exercised at the facade level only until the pipeline
  exposes per-point fields; the manifest is consumed by the CLI next.
