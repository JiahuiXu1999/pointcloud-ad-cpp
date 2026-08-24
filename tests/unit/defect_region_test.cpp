#include "defect_cluster.hpp"
#include "defect_region.hpp"
#include "deviation_field.hpp"

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
using pointcloud_ad::ValidatedComparisonConfig;
using pointcloud_ad::ValidatedDetectionConfig;
using pointcloud_ad::Vec3f;
using pointcloud_ad::comparison::compute_deviation_field;
using pointcloud_ad::detection::apply_severity;
using pointcloud_ad::detection::cluster_deviation_defects;
using pointcloud_ad::detection::measure_deviation_region;
using pointcloud_ad::detection::Severity;
using pointcloud_ad::detection::SeverityRule;

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

bool near(double actual, double expected, double tolerance = 1.0e-9) {
  return std::abs(actual - expected) <= tolerance;
}

struct Patch final {
  OwnedSurface surface;
};

[[nodiscard]] Patch make_patch(const FrameId& frame, double half_span, double step,
                               double amplitude, double sigma) {
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  const double two_sigma_squared = 2.0 * sigma * sigma;
  for (double x = -half_span; x <= half_span + 1.0e-9; x += step) {
    for (double y = -half_span; y <= half_span + 1.0e-9; y += step) {
      const double z = amplitude * std::exp(-(x * x + y * y) / two_sigma_squared);
      points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      normals.push_back({0.0F, 0.0F, 1.0F});
    }
  }
  return Patch{OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                                    LengthUnit::millimeter, frame)
                   .value()};
}

ValidatedComparisonConfig comparison_config() {
  return ValidatedComparisonConfig{2.0, 35.0, 0.6, 0.75};
}

ValidatedDetectionConfig detection_config() {
  return ValidatedDetectionConfig{0.25, -0.25, 0.6, 20, 0.0};
}

} // namespace

int main() {
  bool passed = true;

  const auto reference_frame = FrameId::create("fixture").value();
  const auto scan_frame = FrameId::create("scanner").value();

  const auto reference = make_patch(reference_frame, 5.0, 0.5, 0.0, 1.0);
  const auto bump = make_patch(scan_frame, 5.0, 0.5, 0.8, 1.5);
  const auto comparison = comparison_config();
  const auto detection = detection_config();

  auto field =
      compute_deviation_field(reference.surface.view(), {}, bump.surface.view(), comparison);
  passed &= expect(static_cast<bool>(field), "bump deviation field must be computed");
  if (field) {
    auto clusters = cluster_deviation_defects(bump.surface.view(), field.value(), detection);
    passed &= expect(static_cast<bool>(clusters) && clusters.value().size() == 1U,
                     "bump clustering must produce exactly one cluster");
    if (clusters && clusters.value().size() == 1U) {
      auto region =
          measure_deviation_region(clusters.value()[0], bump.surface.view(), field.value(), 7U);
      passed &= expect(static_cast<bool>(region), "region measurement must succeed");
      if (region) {
        passed &= expect(region.value().id == 7U, "region id must be preserved");
        passed &= expect(region.value().point_count == clusters.value()[0].point_indices.size(),
                         "region point count must match the cluster");
        // The bump is centered at the origin.
        passed &= expect(std::abs(region.value().centroid.x) <= 0.5 &&
                             std::abs(region.value().centroid.y) <= 0.5 &&
                             region.value().centroid.z > 0.0,
                         "bump centroid must be near the origin and above the plane");
        passed &= expect(near(region.value().max_abs_mm, 0.8, 0.05),
                         "bump max absolute deviation must match the injected height");
        passed &= expect(region.value().mean_mm > 0.0, "bump mean deviation must be positive");
        passed &= expect(region.value().aabb_min.z >= 0.0 && region.value().aabb_max.z > 0.0,
                         "bump AABB must span the raised surface");
        passed &= expect(region.value().estimated_area_mm2.has_value() &&
                             *region.value().estimated_area_mm2 > 0.0 &&
                             region.value().area_is_approximate,
                         "bump area must be a positive approximate estimate");
      }
    }
  }

  // Severity rule mapping.
  {
    pointcloud_ad::detection::DefectRegion region;
    region.max_abs_mm = 0.8;
    passed &= expect(apply_severity(region, SeverityRule{{0.5}, {0.2}}) == Severity::reject,
                     "max deviation above the reject bound must map to reject");
    passed &= expect(apply_severity(region, SeverityRule{{1.0}, {0.5}}) == Severity::warning,
                     "max deviation between the bounds must map to warning");
    passed &= expect(apply_severity(region, SeverityRule{{1.0}, {}}) == Severity::info,
                     "max deviation below all bounds must map to info");
  }

  return passed ? 0 : 1;
}
