#include "defect_region.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_ad::detection {
namespace {

[[nodiscard]] Error region_error(ErrorCode code, std::string field, std::string reason) {
  return Error{code,
               PipelineStage::detect,
               "defect region measurement failed",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

[[nodiscard]] bool valid_at(SurfaceView surface, std::size_t index) noexcept {
  return surface.valid().empty() || surface.valid()[index] == 1U;
}

struct PointSample final {
  Vec3d position;
  double signed_mm{};
};

// Collects member positions and, when a deviation field is supplied, signed deviations.
[[nodiscard]] Result<std::vector<PointSample>>
collect_samples(const DefectCluster& cluster, SurfaceView surface,
                const comparison::DeviationField* field) {
  if (surface.unit() != LengthUnit::millimeter) {
    return Result<std::vector<PointSample>>::failure(region_error(
        ErrorCode::invalid_input, "surface.unit", "must be normalized to millimetres"));
  }
  if (field != nullptr && field->size() != surface.storage_size()) {
    return Result<std::vector<PointSample>>::failure(
        region_error(ErrorCode::invalid_input, "field", "must span the surface storage layout"));
  }
  std::vector<PointSample> samples;
  samples.reserve(cluster.point_indices.size());
  for (const std::size_t index : cluster.point_indices) {
    if (index >= surface.storage_size() || !valid_at(surface, index)) {
      return Result<std::vector<PointSample>>::failure(
          region_error(ErrorCode::invalid_input, "cluster", "references an invalid surface index"));
    }
    const Vec3f point = surface.points()[index];
    PointSample sample;
    sample.position = {static_cast<double>(point.x), static_cast<double>(point.y),
                       static_cast<double>(point.z)};
    if (field != nullptr) {
      sample.signed_mm = field->samples()[index].signed_mm;
    }
    samples.push_back(sample);
  }
  return Result<std::vector<PointSample>>::success(std::move(samples));
}

[[nodiscard]] DefectRegion build_region(const DefectCluster& cluster,
                                        std::vector<PointSample> samples, bool has_deviation,
                                        std::uint32_t id) noexcept {
  DefectRegion region;
  region.id = id;
  region.type = cluster.type;
  region.point_count = samples.size();

  Vec3d centroid{};
  Vec3d aabb_min{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()};
  Vec3d aabb_max{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity()};

  for (const PointSample& sample : samples) {
    centroid.x += sample.position.x;
    centroid.y += sample.position.y;
    centroid.z += sample.position.z;
    aabb_min.x = std::min(aabb_min.x, sample.position.x);
    aabb_min.y = std::min(aabb_min.y, sample.position.y);
    aabb_min.z = std::min(aabb_min.z, sample.position.z);
    aabb_max.x = std::max(aabb_max.x, sample.position.x);
    aabb_max.y = std::max(aabb_max.y, sample.position.y);
    aabb_max.z = std::max(aabb_max.z, sample.position.z);
  }
  if (!samples.empty()) {
    const double scale = 1.0 / static_cast<double>(samples.size());
    centroid.x *= scale;
    centroid.y *= scale;
    centroid.z *= scale;
  }
  region.centroid = centroid;
  region.aabb_min = aabb_min;
  region.aabb_max = aabb_max;

  if (has_deviation) {
    std::vector<double> absolute_deviations;
    absolute_deviations.reserve(samples.size());
    double signed_sum = 0.0;
    double squared_sum = 0.0;
    double maximum_absolute = 0.0;
    for (const PointSample& sample : samples) {
      signed_sum += sample.signed_mm;
      squared_sum += sample.signed_mm * sample.signed_mm;
      maximum_absolute = std::max(maximum_absolute, std::abs(sample.signed_mm));
      absolute_deviations.push_back(std::abs(sample.signed_mm));
    }
    if (!samples.empty()) {
      region.mean_mm = signed_sum / static_cast<double>(samples.size());
      region.rms_mm = std::sqrt(squared_sum / static_cast<double>(samples.size()));
      region.max_abs_mm = maximum_absolute;
      std::sort(absolute_deviations.begin(), absolute_deviations.end());
      const std::size_t percentile = static_cast<std::size_t>(
          std::ceil(0.95 * static_cast<double>(absolute_deviations.size())));
      const std::size_t clamped =
          percentile == 0U ? 0U : std::min(percentile - 1U, absolute_deviations.size() - 1U);
      region.p95_abs_mm = absolute_deviations[clamped];
    }
  }

  // Point-cloud area estimate from the mean nearest-neighbour distance among cluster members.
  if (samples.size() >= 2U) {
    double nearest_sum = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
      double nearest = std::numeric_limits<double>::infinity();
      for (std::size_t j = 0; j < samples.size(); ++j) {
        if (i == j) {
          continue;
        }
        const double dx = samples[i].position.x - samples[j].position.x;
        const double dy = samples[i].position.y - samples[j].position.y;
        const double dz = samples[i].position.z - samples[j].position.z;
        nearest = std::min(nearest, std::sqrt(dx * dx + dy * dy + dz * dz));
      }
      nearest_sum += nearest;
    }
    const double mean_nearest = nearest_sum / static_cast<double>(samples.size());
    region.estimated_area_mm2 = mean_nearest * mean_nearest * static_cast<double>(samples.size());
    region.area_is_approximate = true;
  }

  return region;
}

} // namespace

Result<DefectRegion> measure_deviation_region(const DefectCluster& cluster,
                                              SurfaceView aligned_scan,
                                              const comparison::DeviationField& field,
                                              std::uint32_t id) noexcept {
  try {
    auto samples = collect_samples(cluster, aligned_scan, &field);
    if (!samples) {
      return Result<DefectRegion>::failure(std::move(samples).error());
    }
    return Result<DefectRegion>::success(
        build_region(cluster, std::move(samples).value(), true, id));
  } catch (const std::exception& exception) {
    return Result<DefectRegion>::failure(
        region_error(ErrorCode::internal_error, "exception", exception.what()));
  } catch (...) {
    return Result<DefectRegion>::failure(
        region_error(ErrorCode::internal_error, "exception", "unknown exception"));
  }
}

Result<DefectRegion> measure_missing_region(const DefectCluster& cluster, SurfaceView reference,
                                            std::uint32_t id) noexcept {
  try {
    auto samples = collect_samples(cluster, reference, nullptr);
    if (!samples) {
      return Result<DefectRegion>::failure(std::move(samples).error());
    }
    return Result<DefectRegion>::success(
        build_region(cluster, std::move(samples).value(), false, id));
  } catch (const std::exception& exception) {
    return Result<DefectRegion>::failure(
        region_error(ErrorCode::internal_error, "exception", exception.what()));
  } catch (...) {
    return Result<DefectRegion>::failure(
        region_error(ErrorCode::internal_error, "exception", "unknown exception"));
  }
}

Severity apply_severity(const DefectRegion& region, const SeverityRule& rule) noexcept {
  if (rule.reject_max_abs_mm && region.max_abs_mm >= *rule.reject_max_abs_mm) {
    return Severity::reject;
  }
  if (rule.warning_max_abs_mm && region.max_abs_mm >= *rule.warning_max_abs_mm) {
    return Severity::warning;
  }
  return Severity::info;
}

} // namespace pointcloud_ad::detection
