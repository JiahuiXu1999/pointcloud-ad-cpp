#pragma once

#include <pointcloud_ad/registration.hpp>

namespace pointcloud_ad::backends::pcl_backend {

// Runs one deterministic robust point-to-plane ICP solve. The reference and scan surfaces must be
// normalized millimetre views with matching scan-to-reference frame direction and unit-length
// normals on every valid point. PCL types never leave this translation unit; the returned metrics
// are the public backend-neutral contract consumed by the registration gate.
//
// Reference KD-tree construction and correspondence search stay behind this boundary. Translation
// and rotation deltas are reported relative to the initial transform so a downstream gate can bound
// movement without re-deriving it.
[[nodiscard]] Result<RegistrationMetrics>
align_point_to_plane(SurfaceView reference, SurfaceView scan,
                     const RigidTransform& initial_transform,
                     RegistrationParameters parameters) noexcept;

} // namespace pointcloud_ad::backends::pcl_backend
