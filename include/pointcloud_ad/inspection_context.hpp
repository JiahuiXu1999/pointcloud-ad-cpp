#pragma once
#include <cstddef>
#include <memory>
#include <pointcloud_ad/inspection_pipeline.hpp>

namespace pointcloud_ad {
// Owned normalized/preprocessed reference and reusable registration resources for repeated scans.
// create snapshots reference data and configuration; caller buffers may be released afterwards.
// run borrows scan/request synchronously, returns owned results, and never performs file I/O.
// Geometry is normalized to mm; poses are right-handed scan-to-reference. Reference normals must
// meet the same orientation contract as InspectionPipeline. Capacity counts logical scan points.
// One caller at a time per context; independent contexts may run concurrently. Errors (including
// moved-from use, wrong frames and capacity) return Result. Rejected inputs do not invalidate
// reuse.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4251)
#endif
class POINTCLOUD_AD_EXPORT InspectionContext final {
public:
  InspectionContext(const InspectionContext&) = delete;
  InspectionContext& operator=(const InspectionContext&) = delete;
  InspectionContext(InspectionContext&&) noexcept;
  InspectionContext& operator=(InspectionContext&&) noexcept;
  ~InspectionContext();
  [[nodiscard]] static Result<InspectionContext>
  create(InspectionConfig config, SurfaceView reference, std::size_t max_scan_points) noexcept;
  [[nodiscard]] Result<InspectionResult> run(SurfaceView scan,
                                             const InspectionRequest& request = {}) noexcept;

private:
  struct Impl;
  explicit InspectionContext(std::unique_ptr<Impl> impl) noexcept;
  std::unique_ptr<Impl> impl_;
};
#ifdef _MSC_VER
#pragma warning(pop)
#endif
} // namespace pointcloud_ad
