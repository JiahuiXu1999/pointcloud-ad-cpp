# Architecture boundaries

PointCloudAD is a layered library with inward dependencies. The public library is backend-neutral;
PCL is an implementation detail, not a domain model.

```text
                         +-------------------+
                         |        CLI        |
                         +---------+---------+
                                   |
                         +---------v---------+
                         |     Pipeline      |
                         +---+---+---+---+---+
                             |   |   |   |
              +--------------+   |   |   +----------------+
              |                  |   |                    |
        +-----v-----+      +-----v---v--+          +------v------+
        |    I/O    |      | Algorithms |          | Serialization|
        +-----+-----+      +------+-----+          +------+------+
              |                   |                       |
        +-----v-------------------v-----+                 |
        |       Backend facade (PCL)    |                 |
        +---------------+---------------+                 |
                        |                                 |
                  +-----v------+<-------------------------+
                  | Core/model |
                  +------------+
```

The diagram describes permitted dependency direction, not the current M1 file count.

## Public boundary

Only headers installed from `include/pointcloud_ad/` are public. Public values express project
semantics with project-owned types. Backend objects are converted at adapter boundaries.

## Backend boundary

All future PCL includes must be beneath `src/backends/pcl/`. Other modules communicate with that
backend through internal project-owned interfaces. This keeps compile cost, ABI risk, and dependency
leakage contained.

## Current target graph

```text
PointCloudAD::pointcloud_ad  <-  pcad
             ^
             +-------------- pointcloud_ad_unit_tests
             +-------------- result/geometry/config contract tests
             +-------------- private PCL facade conversion tests
             +-------------- installed consumer smoke test
```

`pointcloud_ad` is a shared library. Public declarations opt into the generated export macro; CMake
does not auto-export implementation symbols. On Windows, runtime artifacts share `bin/`, import and
link libraries use `lib/`, and installed consumers deploy the DLL next to their executable.

## M1 public contracts

- `status.hpp` and `result.hpp` define owned, deterministic program failures and a move-only
  value-or-error carrier. Inspection verdicts remain a separate later contract.
- `geometry.hpp` defines explicit length units, project-owned vectors, validated UTF-8 frame IDs,
  and right-handed rigid transforms stored row-major and applied to column vectors in
  source-to-target direction. Transform translations are millimetres.
- `config.hpp` separates raw optional fields from an immutable validated configuration. Production
  thresholds, registration/coverage gates, execution provenance, and a quantified measurement error
  budget must all be present; validation performs no I/O and returns field-addressable errors.

## M2 geometry contracts

- `SurfaceView` borrows immutable point, normal, mask, and frame data; `OwnedSurface` owns immutable
  storage and is created only after validation.
- Organized topology supports explicit element strides and preserves row padding while reporting a
  separate logical point count.
- Validity masks preserve intentionally invalid/non-finite samples; every unmasked sample and its
  optional normal must be finite.
- Normalization converts coordinates once into millimetres, requires explicit frame transforms, and
  rotates normals without translating or scaling them.
- ADR-0002 fixes PCL behind `src/backends/pcl/`; the dependency is not permitted in installed headers
  or direct `src/io/` includes.
- The opaque, move-only backend surface retains PCL storage and an explicit mask sidecar. Conversion
  copies logical points in stable row-major order, compacts organized row padding, and requires units
  and frame metadata when returning to an `OwnedSurface`.
- Backend tests compile the private facade implementation into their own executable. This exercises
  PCL conversions without exporting internal backend symbols from the production DLL.
- `src/io/ply_adapter.*` owns path/format validation and calls a project-owned private backend
  contract. PCL performs PLY parsing only beneath `src/backends/pcl/`; the adapter preserves XYZ,
  optional normals, explicit validity, stable logical order, and organized dimensions while leaving
  unit/frame semantics explicit at the call boundary.
- The sibling PCD adapter uses the same checked field codec, so PLY and PCD agree on XYZ, complete
  normal triples, uint8 masks, finite-value derivation, topology, semantic metadata, and errors.

## M3 preprocessing contracts

- The validity stage accepts only normalized millimetre surfaces and an optional inclusive AABB in
  the same frame. It copies geometry unchanged while producing an explicit mask, logical valid count,
  and one reason for every storage slot.
- Reason precedence is deterministic: non-finite point, non-finite normal, input mask, then ROI.
  Organized row padding is never processed as geometry and is marked separately.
- Voxel sampling uses floor-based signed integer keys and lexicographic key order. Each centroid owns
  a CSR slice of original storage indices in row-major order; invalid samples and organized padding
  never enter a voxel. Coordinates accumulate in double, and optional normals are normalized sums.
- Missing normals use radius-neighborhood PCA behind the PCL facade with sorted neighbor indices and
  an explicit minimum-neighbor gate. Existing normals are normalized through the same result model.
- Orientation proof is separate from normal availability. A direction or viewpoint hint may prove and
  flip a normal; absent evidence leaves orientation unproven so later dent/bump logic must not infer a
  sign. Boundaries remain independent flags: organized inputs use four-neighbor discontinuities and
  unorganized inputs use deterministic tangent-plane angular gaps from PCL radius searches.

## M4 registration contracts

- `registration.hpp` defines backend-neutral registration contracts and never includes PCL or Eigen.
- `RegistrationParameters` carries validated millimetre/radian solver controls; `RegistrationInput`
  borrows the normalized reference and scan surfaces plus a scan-to-reference initial transform and
  validates units, non-empty inputs, frame direction, and parameter ranges.
- `RegistrationConvergence` reports solver-level termination only; it is not an inspection verdict and
  never couples to process exit codes.
- `RegistrationMetrics` owns the final scan-to-reference transform and quantitative fitness, inlier
  RMSE, and translation/rotation deltas measured relative to the initial transform so the later
  quality gate can bound movement without re-deriving it.
- PCAD-REG-001 defines contracts only; ICP/correspondence search (REG-002) and the quality gate
  (REG-003) consume these types and remain unimplemented.
