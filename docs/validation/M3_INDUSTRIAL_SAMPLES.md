# M3 industrial sample validation status

M3 implementation and automated synthetic acceptance are complete. Industrial acceptance is not yet
verified because no real normal-part, dent, bump, or missing-material point clouds were supplied.

## Current evidence

- Fixed deterministic planar clouds validate radius-neighborhood PCA and minimum support.
- Explicit masks, non-finite samples, ROI bounds, organized padding, and voxel source mapping are
  covered by unit tests.
- Fixed orientation hints cover proven direction/viewpoint cases and the unproven-sign case.
- Synthetic organized depth jumps and unorganized angular gaps cover boundary behavior.

## Samples required to close industrial validation

- A representative normal part in the production fixture/scanner frames.
- Known dent and bump samples with measurement references and viewpoint/orientation metadata.
- A missing-material sample for later coverage validation.
- Acquisition units, expected ROI, sensor viewpoint or outward direction, point spacing, and the
  intended normal/boundary radii for every sample.

Until those inputs are available, no synthetic threshold or orientation hint is presented as a
production default, and M3 industrial acceptance remains explicitly unverified.
