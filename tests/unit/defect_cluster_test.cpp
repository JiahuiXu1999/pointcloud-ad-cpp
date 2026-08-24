#include "defect_cluster.hpp"
#include "deviation_field.hpp"

#include <algorithm>
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
using pointcloud_ad::ValidatedDetectionConfig;
using pointcloud_ad::Vec3d;
using pointcloud_ad::Vec3f;
using pointcloud_ad::comparison::compute_deviation_field;
using pointcloud_ad::comparison::DeviationReason;
using pointcloud_ad::detection::cluster_deviation_defects;
using pointcloud_ad::detection::DefectType;

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

struct Patch final {
  OwnedSurface surface;
};

// z = amplitude * exp(-(x^2 + y^2) / (2 sigma^2)) with unit +z normals; amplitude 0 gives the flat
// reference plane.
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
  const auto comparison = comparison_config();
  const auto detection = detection_config();

  // AC-003: a bump classifies as a single bump cluster whose members all have positive deviation.
  {
    const auto bump = make_patch(scan_frame, 5.0, 0.5, 0.8, 1.5);
    auto field =
        compute_deviation_field(reference.surface.view(), {}, bump.surface.view(), comparison);
    passed &= expect(static_cast<bool>(field), "bump deviation field must be computed");
    if (field) {
      auto clusters = cluster_deviation_defects(bump.surface.view(), field.value(), detection);
      passed &= expect(static_cast<bool>(clusters), "bump clustering must succeed");
      if (clusters) {
        passed &=
            expect(clusters.value().size() == 1U && clusters.value()[0].type == DefectType::bump,
                   "a single bump must form exactly one bump cluster");
        if (clusters.value().size() == 1U) {
          const auto& samples = field.value().samples();
          bool all_positive = true;
          for (const std::size_t index : clusters.value()[0].point_indices) {
            all_positive = all_positive && samples[index].reason == DeviationReason::valid &&
                           samples[index].signed_mm > detection.positive_threshold_mm;
          }
          passed &=
              expect(all_positive,
                     "every bump cluster member must have positive deviation above threshold");
        }
      }
    }
  }

  // AC-004: a dent classifies as a single dent cluster whose members all have negative deviation.
  {
    const auto dent = make_patch(scan_frame, 5.0, 0.5, -0.8, 1.5);
    auto field =
        compute_deviation_field(reference.surface.view(), {}, dent.surface.view(), comparison);
    passed &= expect(static_cast<bool>(field), "dent deviation field must be computed");
    if (field) {
      auto clusters = cluster_deviation_defects(dent.surface.view(), field.value(), detection);
      passed &= expect(static_cast<bool>(clusters), "dent clustering must succeed");
      if (clusters) {
        passed &=
            expect(clusters.value().size() == 1U && clusters.value()[0].type == DefectType::dent,
                   "a single dent must form exactly one dent cluster");
        if (clusters.value().size() == 1U) {
          const auto& samples = field.value().samples();
          bool all_negative = true;
          for (const std::size_t index : clusters.value()[0].point_indices) {
            all_negative = all_negative && samples[index].reason == DeviationReason::valid &&
                           samples[index].signed_mm < detection.negative_threshold_mm;
          }
          passed &=
              expect(all_negative,
                     "every dent cluster member must have negative deviation below threshold");
        }
      }
    }
  }

  // A clean surface must produce no clusters.
  {
    auto field =
        compute_deviation_field(reference.surface.view(), {}, reference.surface.view(), comparison);
    passed &= expect(static_cast<bool>(field), "clean deviation field must be computed");
    if (field) {
      auto clusters = cluster_deviation_defects(reference.surface.view(), field.value(), detection);
      passed &= expect(static_cast<bool>(clusters) && clusters.value().empty(),
                       "a clean surface must produce no defect clusters");
    }
  }

  // A tiny bump below the minimum cluster size must be filtered out.
  {
    const auto tiny = make_patch(scan_frame, 5.0, 0.5, 0.8, 0.3);
    auto field =
        compute_deviation_field(reference.surface.view(), {}, tiny.surface.view(), comparison);
    passed &= expect(static_cast<bool>(field), "tiny-bump deviation field must be computed");
    if (field) {
      auto clusters = cluster_deviation_defects(tiny.surface.view(), field.value(), detection);
      passed &= expect(static_cast<bool>(clusters) && clusters.value().empty(),
                       "a bump below the minimum cluster size must be filtered out");
    }
  }

  return passed ? 0 : 1;
}
