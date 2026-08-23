#include "deviation_field.hpp"

#include "pcl_comparison_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_ad::comparison {
namespace {

constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] Error deviation_error(ErrorCode code, std::string field, std::string reason) {
  return Error{code,
               PipelineStage::compare,
               "deviation field computation failed",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

[[nodiscard]] bool valid_at(SurfaceView surface, std::size_t index) noexcept {
  return surface.valid().empty() || surface.valid()[index] == 1U;
}

[[nodiscard]] double vector_length(Vec3f vector) noexcept {
  return std::sqrt(static_cast<double>(vector.x) * vector.x +
                   static_cast<double>(vector.y) * vector.y +
                   static_cast<double>(vector.z) * vector.z);
}

[[nodiscard]] double dot(Vec3f left, Vec3f right) noexcept {
  return static_cast<double>(left.x) * right.x + static_cast<double>(left.y) * right.y +
         static_cast<double>(left.z) * right.z;
}

template <typename Function> void for_each_logical_index(SurfaceView surface, Function&& function) {
  if (!surface.grid()) {
    for (std::size_t index = 0; index < surface.storage_size(); ++index) {
      function(index);
    }
    return;
  }
  const auto& grid = *surface.grid();
  for (std::size_t row = 0; row < grid.height; ++row) {
    for (std::size_t column = 0; column < grid.width; ++column) {
      function(row * static_cast<std::size_t>(grid.row_stride) + column);
    }
  }
}

[[nodiscard]] Result<DeviationField> compute_impl(SurfaceView reference,
                                                  std::span<const std::uint8_t> reference_boundary,
                                                  SurfaceView aligned_scan,
                                                  const ValidatedComparisonConfig& config) {
  if (reference.unit() != LengthUnit::millimeter) {
    return Result<DeviationField>::failure(deviation_error(
        ErrorCode::invalid_input, "reference.unit", "must be normalized to millimetres"));
  }
  if (aligned_scan.unit() != LengthUnit::millimeter) {
    return Result<DeviationField>::failure(deviation_error(
        ErrorCode::invalid_input, "aligned_scan.unit", "must be normalized to millimetres"));
  }
  if (reference.normals().empty()) {
    return Result<DeviationField>::failure(deviation_error(
        ErrorCode::invalid_input, "reference.normals", "are required to compute signed deviation"));
  }
  if (!reference_boundary.empty() && reference_boundary.size() != reference.storage_size()) {
    return Result<DeviationField>::failure(deviation_error(
        ErrorCode::invalid_input, "reference_boundary", "must span the reference storage layout"));
  }

  auto nearest = backends::pcl_backend::nearest_neighbors(reference, aligned_scan,
                                                          config.max_search_distance_mm);
  if (!nearest) {
    return Result<DeviationField>::failure(std::move(nearest).error());
  }
  const auto& neighbor_index = nearest.value().neighbor_index;
  const auto& distances = nearest.value().distance_mm;

  const bool scan_has_normals = !aligned_scan.normals().empty();
  const double max_normal_cosine = std::cos(config.max_normal_angle_deg * kPi / 180.0);

  std::vector<DeviationSample> samples(aligned_scan.storage_size());
  std::size_t valid_count = 0U;
  for_each_logical_index(aligned_scan, [&](std::size_t index) {
    DeviationSample& sample = samples[index];
    if (!valid_at(aligned_scan, index)) {
      sample.reason = DeviationReason::input_invalid;
      return;
    }
    const auto neighbor = neighbor_index[index];
    if (neighbor < 0) {
      sample.reason = DeviationReason::no_neighbor;
      return;
    }
    const auto reference_storage = static_cast<std::size_t>(neighbor);
    const Vec3f reference_normal = reference.normals()[reference_storage];
    if (!(vector_length(reference_normal) > 0.0) ||
        !std::isfinite(vector_length(reference_normal))) {
      sample.reason = DeviationReason::normal_missing;
      return;
    }
    const Vec3f scan_point = aligned_scan.points()[index];
    const Vec3f reference_point = reference.points()[reference_storage];
    const Vec3f delta{scan_point.x - reference_point.x, scan_point.y - reference_point.y,
                      scan_point.z - reference_point.z};

    double normal_angle = 0.0;
    if (scan_has_normals) {
      const Vec3f scan_normal = aligned_scan.normals()[index];
      if (vector_length(scan_normal) > 0.0) {
        const double length_product = vector_length(reference_normal) * vector_length(scan_normal);
        const double cosine =
            std::clamp(dot(reference_normal, scan_normal) / length_product, -1.0, 1.0);
        normal_angle = std::acos(cosine) * 180.0 / kPi;
        if (cosine < max_normal_cosine) {
          sample.reason = DeviationReason::normal_mismatch;
          sample.normal_angle_deg = normal_angle;
          sample.euclidean_mm = static_cast<double>(distances[index]);
          return;
        }
      }
    }
    if (!reference_boundary.empty() && reference_boundary[reference_storage] != 0U) {
      sample.reason = DeviationReason::reference_boundary;
      sample.euclidean_mm = static_cast<double>(distances[index]);
      sample.normal_angle_deg = normal_angle;
      return;
    }

    const double reference_length = vector_length(reference_normal);
    sample.euclidean_mm = static_cast<double>(distances[index]);
    sample.signed_mm = dot(reference_normal, delta) / reference_length;
    sample.normal_angle_deg = normal_angle;
    sample.reason = DeviationReason::valid;
    ++valid_count;
  });

  return Result<DeviationField>::success(DeviationField(std::move(samples), valid_count));
}

} // namespace

Result<DeviationField> compute_deviation_field(SurfaceView reference,
                                               std::span<const std::uint8_t> reference_boundary,
                                               SurfaceView aligned_scan,
                                               const ValidatedComparisonConfig& config) noexcept {
  try {
    return compute_impl(reference, reference_boundary, aligned_scan, config);
  } catch (const std::exception& exception) {
    return Result<DeviationField>::failure(
        deviation_error(ErrorCode::internal_error, "exception", exception.what()));
  } catch (...) {
    return Result<DeviationField>::failure(
        deviation_error(ErrorCode::internal_error, "exception", "unknown exception"));
  }
}

} // namespace pointcloud_ad::comparison
