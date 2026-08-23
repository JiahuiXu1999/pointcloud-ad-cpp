# PCAD-REG-002: robust point-to-plane ICP

## Identity

- Task ID: PCAD-REG-002
- Milestone: M4 / B12
- Objective: Implement the deterministic robust point-to-plane ICP solver behind the PCL backend
  boundary, consuming the backend-neutral registration contracts from PCAD-REG-001.
- Dependencies: PCAD-REG-001, PCAD-BE-001

## Allowed scope

- Files/modules that may change: `src/backends/pcl/pcl_registration_backend.{hpp,cpp}`, its unit
  test, build files, and M4 documentation.
- Public API changes: none. The public `RegistrationParameters`, `RegistrationInput`,
  `RegistrationConvergence`, and `RegistrationMetrics` types from PCAD-REG-001 are consumed as-is.
- New dependencies: none. The solver uses only PCL KD-tree search and Eigen (already permitted
  behind `src/backends/pcl/` and the implementation-only linear-algebra decision D-005).

## Contract

- Inputs and ownership: `align_point_to_plane` borrows millimetre `SurfaceView` reference and scan,
  a scan-to-reference `RigidTransform` initial guess, and validated `RegistrationParameters`. The
  caller keeps both surfaces alive for the call.
- Outputs and ownership: `RegistrationMetrics` owns the final scan-to-reference transform, the
  solver's neutral convergence status, iteration count, valid-pair count, fitness, inlier RMS
  residual, and the translation/rotation deltas relative to the initial guess.
- Units and coordinate frames: all distances in millimetres, rotation deltas in degrees, transforms
  scan-to-reference with double-precision accumulation.
- Determinism requirements: reference KD-tree is built once per call; correspondences are visited
  in storage order; the 6x6 normal equations are accumulated in fixed order; no randomized step.
- Error cases: missing reference normals, empty valid point sets, and a non-representable final
  transform all return `Result` failures; a solver that cannot form a least-squares system reports
  `RegistrationConvergence::degenerate_input` through the metrics value, not a `Result` error.
- Performance budget: one KD-tree build plus `max_iterations` nearest-neighbour passes; per-iteration
  cost is linear in the scan valid point count.

## Exclusions

- Explicitly out of scope: the registration quality gate and any verdict (PCAD-REG-003), global
  registration, alternate backends, and any inspection status or process exit-code coupling.
- Architecture boundaries that must not move: no PCL or Eigen type may enter
  `include/pointcloud_ad/`; the public error model remains `Result<T>`.

## Acceptance

- Required unit tests: a synthetic rigid-alignment case (AC-002) asserting convergence and
  translation error ≤ 0.02 mm and rotation error ≤ 0.02 degrees, plus a grossly-wrong initial pose
  (AC-007) asserting the downstream gate rejects the result.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the new ICP test, the registration-gate test, and all pre-existing tests pass.
- Remaining risks: the metric fields are consumed by PCAD-REG-003 and later pipeline stages; the
  solver assumes unit-length reference normals as produced by the M3 normal stage.
