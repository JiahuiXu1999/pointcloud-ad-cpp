#pragma once

#include "defect_cluster.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/surface.hpp>

namespace pointcloud_ad::detection {

enum class Severity : std::uint8_t { info, warning, reject };

// Measured summary of one defect cluster. Deviation statistics (mean/rms/max/p95) describe the
// signed deviation for dent/bump regions and are zero for missing-material regions. The area is a
// point-cloud estimate and is always flagged approximate.
struct DefectRegion final {
  std::uint32_t id{};
  DefectType type{DefectType::dent};
  std::size_t point_count{};
  Vec3d centroid{};
  Vec3d aabb_min{};
  Vec3d aabb_max{};
  double max_abs_mm{};
  double mean_mm{};
  double rms_mm{};
  double p95_abs_mm{};
  std::optional<double> estimated_area_mm2{};
  bool area_is_approximate{true};
};

// Measures a dent/bump cluster using its member positions and signed deviations.
[[nodiscard]] Result<DefectRegion> measure_deviation_region(const DefectCluster& cluster,
                                                            SurfaceView aligned_scan,
                                                            const comparison::DeviationField& field,
                                                            std::uint32_t id) noexcept;

// Measures a missing-material cluster using member positions only.
[[nodiscard]] Result<DefectRegion> measure_missing_region(const DefectCluster& cluster,
                                                          SurfaceView reference,
                                                          std::uint32_t id) noexcept;

// User-configurable severity rule keyed on maximum absolute deviation. Thresholds are optional so a
// region falls back to the lower severity when a bound is absent.
struct SeverityRule final {
  std::optional<double> reject_max_abs_mm;
  std::optional<double> warning_max_abs_mm;
};

[[nodiscard]] Severity apply_severity(const DefectRegion& region,
                                      const SeverityRule& rule) noexcept;

} // namespace pointcloud_ad::detection
