#include "defect_cluster.hpp"

#include "pcl_cluster_backend.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pointcloud_ad::detection {
namespace {

[[nodiscard]] Error detection_error(ErrorCode code, std::string field, std::string reason) {
  return Error{code,
               PipelineStage::detect,
               "defect clustering failed",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

// Clusters one class of candidates and appends clusters meeting the minimum point count.
[[nodiscard]] Result<std::vector<DefectCluster>>
cluster_class(SurfaceView aligned_scan, const comparison::DeviationField& field, DefectType type,
              double threshold_mm, bool positive, double cluster_tolerance_mm,
              std::uint32_t min_cluster_points) {
  std::vector<std::size_t> candidates;
  const auto samples = field.samples();
  for (std::size_t index = 0; index < samples.size(); ++index) {
    if (samples[index].reason != comparison::DeviationReason::valid) {
      continue;
    }
    const bool exceeds = positive ? samples[index].signed_mm > threshold_mm
                                  : samples[index].signed_mm < threshold_mm;
    if (exceeds) {
      candidates.push_back(index);
    }
  }

  if (candidates.empty()) {
    return Result<std::vector<DefectCluster>>::success({});
  }

  auto labels =
      backends::pcl_backend::euclidean_cluster(aligned_scan, candidates, cluster_tolerance_mm);
  if (!labels) {
    return Result<std::vector<DefectCluster>>::failure(std::move(labels).error());
  }

  std::unordered_map<std::int32_t, std::vector<std::size_t>> groups;
  for (std::size_t candidate = 0; candidate < candidates.size(); ++candidate) {
    groups[labels.value()[candidate]].push_back(candidates[candidate]);
  }

  std::vector<DefectCluster> clusters;
  for (auto& [label, indices] : groups) {
    (void)label;
    if (indices.size() < min_cluster_points) {
      continue;
    }
    DefectCluster cluster;
    cluster.type = type;
    cluster.point_indices = std::move(indices);
    clusters.push_back(std::move(cluster));
  }
  return Result<std::vector<DefectCluster>>::success(std::move(clusters));
}

[[nodiscard]] Result<std::vector<DefectCluster>>
cluster_impl(SurfaceView aligned_scan, const comparison::DeviationField& field,
             const ValidatedDetectionConfig& config) {
  if (aligned_scan.unit() != LengthUnit::millimeter) {
    return Result<std::vector<DefectCluster>>::failure(detection_error(
        ErrorCode::invalid_input, "aligned_scan.unit", "must be normalized to millimetres"));
  }
  if (field.size() != aligned_scan.storage_size()) {
    return Result<std::vector<DefectCluster>>::failure(detection_error(
        ErrorCode::invalid_input, "field", "must span the aligned scan storage layout"));
  }

  auto bumps = cluster_class(aligned_scan, field, DefectType::bump, config.positive_threshold_mm,
                             true, config.cluster_tolerance_mm, config.min_cluster_points);
  if (!bumps) {
    return bumps;
  }
  auto dents = cluster_class(aligned_scan, field, DefectType::dent, config.negative_threshold_mm,
                             false, config.cluster_tolerance_mm, config.min_cluster_points);
  if (!dents) {
    return dents;
  }

  std::vector<DefectCluster> clusters = std::move(bumps.value());
  auto dent_clusters = std::move(dents.value());
  clusters.insert(clusters.end(), std::make_move_iterator(dent_clusters.begin()),
                  std::make_move_iterator(dent_clusters.end()));
  return Result<std::vector<DefectCluster>>::success(std::move(clusters));
}

} // namespace

Result<std::vector<DefectCluster>>
cluster_deviation_defects(SurfaceView aligned_scan, const comparison::DeviationField& field,
                          const ValidatedDetectionConfig& config) noexcept {
  try {
    return cluster_impl(aligned_scan, field, config);
  } catch (const std::exception& exception) {
    return Result<std::vector<DefectCluster>>::failure(
        detection_error(ErrorCode::internal_error, "exception", exception.what()));
  } catch (...) {
    return Result<std::vector<DefectCluster>>::failure(
        detection_error(ErrorCode::internal_error, "exception", "unknown exception"));
  }
}

Result<std::vector<DefectCluster>>
cluster_missing_material(SurfaceView reference, const comparison::CoverageField& field,
                         const ValidatedDetectionConfig& config) noexcept {
  try {
    if (reference.unit() != LengthUnit::millimeter) {
      return Result<std::vector<DefectCluster>>::failure(detection_error(
          ErrorCode::invalid_input, "reference.unit", "must be normalized to millimetres"));
    }
    if (field.size() != reference.storage_size()) {
      return Result<std::vector<DefectCluster>>::failure(detection_error(
          ErrorCode::invalid_input, "field", "must span the reference storage layout"));
    }

    std::vector<std::size_t> candidates;
    const auto samples = field.samples();
    for (std::size_t index = 0; index < samples.size(); ++index) {
      if (samples[index].reason == comparison::CoverageReason::no_neighbor) {
        candidates.push_back(index);
      }
    }
    if (candidates.empty()) {
      return Result<std::vector<DefectCluster>>::success({});
    }

    auto labels = backends::pcl_backend::euclidean_cluster(reference, candidates,
                                                           config.cluster_tolerance_mm);
    if (!labels) {
      return Result<std::vector<DefectCluster>>::failure(std::move(labels).error());
    }

    std::unordered_map<std::int32_t, std::vector<std::size_t>> groups;
    for (std::size_t candidate = 0; candidate < candidates.size(); ++candidate) {
      groups[labels.value()[candidate]].push_back(candidates[candidate]);
    }

    std::vector<DefectCluster> clusters;
    for (auto& [label, indices] : groups) {
      (void)label;
      if (indices.size() < config.min_cluster_points) {
        continue;
      }
      DefectCluster cluster;
      cluster.type = DefectType::missing_material;
      cluster.point_indices = std::move(indices);
      clusters.push_back(std::move(cluster));
    }
    return Result<std::vector<DefectCluster>>::success(std::move(clusters));
  } catch (const std::exception& exception) {
    return Result<std::vector<DefectCluster>>::failure(
        detection_error(ErrorCode::internal_error, "exception", exception.what()));
  } catch (...) {
    return Result<std::vector<DefectCluster>>::failure(
        detection_error(ErrorCode::internal_error, "exception", "unknown exception"));
  }
}

} // namespace pointcloud_ad::detection
