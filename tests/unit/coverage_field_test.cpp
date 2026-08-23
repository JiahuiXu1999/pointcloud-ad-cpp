#include "coverage_field.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/surface.hpp>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using pointcloud_ad::FrameId;
using pointcloud_ad::LengthUnit;
using pointcloud_ad::OwnedSurface;
using pointcloud_ad::SurfaceView;
using pointcloud_ad::ValidatedComparisonConfig;
using pointcloud_ad::Vec3f;
using pointcloud_ad::comparison::compute_coverage_field;
using pointcloud_ad::comparison::CoverageReason;

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

// A z=0 plane sampled on a uniform grid. `predicate` selects which (x, y) grid positions are
// emitted, so callers can punch holes or keep half planes.
[[nodiscard]] OwnedSurface make_plane(const FrameId& frame, double half_span, double step,
                                      bool (*predicate)(double, double)) {
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  for (double x = -half_span; x <= half_span + 1.0e-9; x += step) {
    for (double y = -half_span; y <= half_span + 1.0e-9; y += step) {
      if (!predicate(x, y)) {
        continue;
      }
      points.push_back({static_cast<float>(x), static_cast<float>(y), 0.0F});
      normals.push_back({0.0F, 0.0F, 1.0F});
    }
  }
  return OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                              LengthUnit::millimeter, frame)
      .value();
}

bool all(double, double) {
  return true;
}

bool outside_hole(double x, double y) {
  return !(std::abs(x) < 3.0 && std::abs(y) < 3.0);
}

bool core_of_hole(double x, double y) {
  return std::abs(x) <= 1.5 && std::abs(y) <= 1.5;
}

bool left_half(double x, double) {
  return x < 0.0;
}

ValidatedComparisonConfig comparison_config() {
  return ValidatedComparisonConfig{1.0, 35.0, 0.6, 0.75};
}

} // namespace

int main() {
  bool passed = true;

  const auto reference_frame = FrameId::create("fixture").value();
  const auto scan_frame = FrameId::create("scanner").value();
  const auto config = comparison_config();

  const auto reference = make_plane(reference_frame, 5.0, 0.5, all);

  // AC-005: a missing block in the scan must leave the reference's hole as uncovered (no_neighbor)
  // while the surrounding surface stays covered.
  {
    const auto scan = make_plane(scan_frame, 5.0, 0.5, outside_hole);
    auto field = compute_coverage_field(reference.view(), scan.view(), {}, config);
    passed &= expect(static_cast<bool>(field), "coverage field must be computed");
    if (field) {
      std::size_t core_uncovered = 0U;
      std::size_t core_total = 0U;
      std::size_t outer_covered = 0U;
      std::size_t outer_total = 0U;
      const auto points = reference.view().points();
      for (std::size_t index = 0; index < field.value().size(); ++index) {
        const Vec3f point = points[index];
        const auto reason = field.value().samples()[index].reason;
        if (core_of_hole(point.x, point.y)) {
          ++core_total;
          core_uncovered += reason == CoverageReason::no_neighbor ? 1U : 0U;
        } else if (std::abs(point.x) > 4.0 || std::abs(point.y) > 4.0) {
          ++outer_total;
          outer_covered += reason == CoverageReason::covered ? 1U : 0U;
        }
      }
      passed &= expect(core_total > 0U && core_uncovered == core_total,
                       "hole core must be entirely uncovered");
      passed &= expect(outer_total > 0U && outer_covered == outer_total,
                       "surface outside the hole must stay covered");
    }
  }

  // AC-006: a scan that only covers half the reference must yield a coverage ratio below the
  // configured minimum, so the coverage gate can reject it.
  {
    const auto scan = make_plane(scan_frame, 5.0, 0.5, left_half);
    auto field = compute_coverage_field(reference.view(), scan.view(), {}, config);
    passed &= expect(static_cast<bool>(field), "partial-scan coverage field must be computed");
    if (field) {
      passed &= expect(field.value().coverage_ratio() < config.min_valid_coverage_ratio,
                       "half coverage must fall below the minimum coverage ratio");
      if (!(field.value().coverage_ratio() < config.min_valid_coverage_ratio)) {
        std::cerr << "  coverage_ratio=" << field.value().coverage_ratio() << '\n';
      }
    }
  }

  // A full scan must report near-complete coverage.
  {
    const auto scan = make_plane(scan_frame, 5.0, 0.5, all);
    auto field = compute_coverage_field(reference.view(), scan.view(), {}, config);
    passed &= expect(static_cast<bool>(field), "full-scan coverage field must be computed");
    if (field) {
      passed &= expect(field.value().coverage_ratio() >= config.min_valid_coverage_ratio,
                       "a full scan must meet the minimum coverage ratio");
    }
  }

  return passed ? 0 : 1;
}
