#include "validity.hpp"

#include <cmath>
#include <exception>
#include <string>
#include <utility>

namespace pointcloud_ad::preprocess {
namespace {

[[nodiscard]] Error preprocess_error(ErrorCode code, std::string field, std::string reason) {
  return Error{code,
               PipelineStage::preprocess,
               "validity preprocessing failed",
               {{"field", std::move(field)}, {"reason", std::move(reason)}}};
}

[[nodiscard]] bool finite(Vec3f value) noexcept {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] bool valid_roi(const AxisAlignedRoi& roi) noexcept {
  return finite(roi.minimum_mm) && finite(roi.maximum_mm) && roi.minimum_mm.x <= roi.maximum_mm.x &&
         roi.minimum_mm.y <= roi.maximum_mm.y && roi.minimum_mm.z <= roi.maximum_mm.z;
}

[[nodiscard]] bool inside(const AxisAlignedRoi& roi, Vec3f point) noexcept {
  return point.x >= roi.minimum_mm.x && point.x <= roi.maximum_mm.x &&
         point.y >= roi.minimum_mm.y && point.y <= roi.maximum_mm.y &&
         point.z >= roi.minimum_mm.z && point.z <= roi.maximum_mm.z;
}

[[nodiscard]] Result<ValidityMaskResult>
apply_validity_mask_impl(SurfaceView surface, const std::optional<AxisAlignedRoi>& roi) {
  if (surface.unit() != LengthUnit::millimeter) {
    return Result<ValidityMaskResult>::failure(preprocess_error(
        ErrorCode::invalid_input, "surface.unit", "must be normalized to millimetres"));
  }
  if (roi && !valid_roi(*roi)) {
    return Result<ValidityMaskResult>::failure(preprocess_error(
        ErrorCode::invalid_argument, "roi", "bounds must be finite and minimum <= maximum"));
  }

  std::vector<Vec3f> points(surface.points().begin(), surface.points().end());
  std::vector<Vec3f> normals(surface.normals().begin(), surface.normals().end());
  std::vector<std::uint8_t> mask(surface.storage_size(), 0U);
  std::vector<InvalidReason> reasons(surface.storage_size(), InvalidReason::row_padding);
  std::size_t valid_count = 0U;

  const auto classify = [&](std::size_t index) {
    const auto point = surface.points()[index];
    const bool input_valid = surface.valid().empty() || surface.valid()[index] == 1U;
    InvalidReason reason = InvalidReason::none;
    if (!finite(point)) {
      reason = InvalidReason::non_finite_point;
    } else if (!surface.normals().empty() && !finite(surface.normals()[index])) {
      reason = InvalidReason::non_finite_normal;
    } else if (!input_valid) {
      reason = InvalidReason::input_masked;
    } else if (roi && !inside(*roi, point)) {
      reason = InvalidReason::outside_roi;
    }
    reasons[index] = reason;
    if (reason == InvalidReason::none) {
      mask[index] = 1U;
      ++valid_count;
    }
  };

  if (!surface.grid()) {
    for (std::size_t index = 0; index < surface.storage_size(); ++index) {
      classify(index);
    }
  } else {
    const auto& grid = *surface.grid();
    for (std::size_t row = 0; row < grid.height; ++row) {
      for (std::size_t column = 0; column < grid.width; ++column) {
        classify(row * static_cast<std::size_t>(grid.row_stride) + column);
      }
    }
  }

  auto owned = OwnedSurface::create(std::move(points), std::move(normals), std::move(mask),
                                    surface.grid(), surface.unit(), surface.frame());
  if (!owned) {
    return Result<ValidityMaskResult>::failure(std::move(owned).error());
  }
  return Result<ValidityMaskResult>::success(
      ValidityMaskResult(std::move(owned).value(), std::move(reasons), valid_count));
}

} // namespace

Result<ValidityMaskResult> apply_validity_mask(SurfaceView surface,
                                               std::optional<AxisAlignedRoi> roi) noexcept {
  try {
    return apply_validity_mask_impl(surface, roi);
  } catch (const std::exception& exception) {
    return Result<ValidityMaskResult>::failure(
        preprocess_error(ErrorCode::internal_error, "exception", exception.what()));
  } catch (...) {
    return Result<ValidityMaskResult>::failure(
        preprocess_error(ErrorCode::internal_error, "exception", "unknown exception"));
  }
}

} // namespace pointcloud_ad::preprocess
