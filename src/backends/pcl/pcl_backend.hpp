#pragma once

#include <memory>
#include <pointcloud_ad/surface.hpp>

namespace pointcloud_ad::backends::pcl_backend {

// Owns backend state without exposing PCL, Eigen, Boost, or filesystem types. The object is
// move-only so its implementation remains private to src/backends/pcl/.
class PclSurface final {
public:
  PclSurface(const PclSurface&) = delete;
  PclSurface& operator=(const PclSurface&) = delete;
  PclSurface(PclSurface&&) noexcept;
  PclSurface& operator=(PclSurface&&) noexcept;
  ~PclSurface();

private:
  struct Impl;

  explicit PclSurface(std::unique_ptr<Impl> implementation) noexcept;

  std::unique_ptr<Impl> implementation_;

  friend Result<PclSurface> to_pcl_surface(SurfaceView surface) noexcept;
  friend Result<OwnedSurface> from_pcl_surface(const PclSurface& surface, LengthUnit unit,
                                               FrameId frame) noexcept;
};

// Converts logical points in deterministic row-major order. Organized row padding is compacted;
// the backend cloud retains width and height with row_stride == width.
[[nodiscard]] Result<PclSurface> to_pcl_surface(SurfaceView surface) noexcept;

// Creates an immutable project-owned surface. Units and frame are explicit because backend clouds
// carry neither semantic; no unit conversion is performed at this boundary.
[[nodiscard]] Result<OwnedSurface> from_pcl_surface(const PclSurface& surface, LengthUnit unit,
                                                    FrameId frame) noexcept;

} // namespace pointcloud_ad::backends::pcl_backend
