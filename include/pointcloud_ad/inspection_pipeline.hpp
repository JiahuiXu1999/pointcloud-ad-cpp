#pragma once

#include <memory>
#include <optional>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/export.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/inspection_result.hpp>
#include <pointcloud_ad/result.hpp>
#include <pointcloud_ad/surface.hpp>
#include <string>

namespace pointcloud_ad {

// Optional per-run overrides. The initial pose is a scan-to-reference transform; when absent, the
// identity pose is used and recorded as a warning diagnostic.
struct InspectionRequest final {
  std::optional<RigidTransform> initial_pose;
  std::string run_id;
};

// The single orchestration entry point. `create` validates the configuration once; `run` performs
// the full normalize -> preprocess -> register -> gate -> transform -> compare -> coverage-gate ->
// detect -> finalize sequence and returns a completed report. `run` never writes files, reads the
// environment, or mutates global state; serialization is the caller's responsibility.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4251)
#endif
class POINTCLOUD_AD_EXPORT InspectionPipeline final {
public:
  InspectionPipeline(const InspectionPipeline&) = delete;
  InspectionPipeline& operator=(const InspectionPipeline&) = delete;
  InspectionPipeline(InspectionPipeline&&) noexcept = default;
  InspectionPipeline& operator=(InspectionPipeline&&) noexcept = default;
  ~InspectionPipeline();

  [[nodiscard]] static Result<InspectionPipeline> create(InspectionConfig config) noexcept;

  [[nodiscard]] Result<InspectionResult> run(SurfaceView reference, SurfaceView scan,
                                             const InspectionRequest& request = {}) const noexcept;

private:
  struct Impl;
  explicit InspectionPipeline(ValidatedInspectionConfig config);

  std::unique_ptr<const Impl> impl_;
};
#ifdef _MSC_VER
#pragma warning(pop)
#endif

} // namespace pointcloud_ad
