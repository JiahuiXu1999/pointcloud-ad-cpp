#include "deviation_field.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
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
using pointcloud_ad::Vec3d;
using pointcloud_ad::Vec3f;
using pointcloud_ad::comparison::compute_deviation_field;
using pointcloud_ad::comparison::DeviationReason;

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

// Deterministic flat patch in the z=0 plane with a smooth Gaussian bump/dent centered at the
// origin. Heights are signed; normals are unit +z (the gentle slope stays well inside the 35-degree
// correspondence angle used by the tests).
struct Patch final {
  OwnedSurface surface;
  std::vector<Vec3d> points;
};

[[nodiscard]] Patch make_patch(const FrameId& frame, double half_span, double step,
                               double amplitude, double sigma) {
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  std::vector<Vec3d> points_double;
  const double two_sigma_squared = 2.0 * sigma * sigma;
  for (double x = -half_span; x <= half_span + 1.0e-9; x += step) {
    for (double y = -half_span; y <= half_span + 1.0e-9; y += step) {
      const double z = amplitude * std::exp(-(x * x + y * y) / two_sigma_squared);
      points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      normals.push_back({0.0F, 0.0F, 1.0F});
      points_double.push_back({x, y, z});
    }
  }
  auto surface = OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                                      LengthUnit::millimeter, frame);
  return Patch{std::move(surface).value(), std::move(points_double)};
}

ValidatedComparisonConfig comparison_config() {
  return ValidatedComparisonConfig{2.0, 35.0, 0.6, 0.75};
}

double maximum_signed(const pointcloud_ad::comparison::DeviationField& field) {
  double maximum = -std::numeric_limits<double>::infinity();
  for (const auto& sample : field.samples()) {
    if (sample.reason == DeviationReason::valid) {
      maximum = std::max(maximum, sample.signed_mm);
    }
  }
  return maximum;
}

double minimum_signed(const pointcloud_ad::comparison::DeviationField& field) {
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto& sample : field.samples()) {
    if (sample.reason == DeviationReason::valid) {
      minimum = std::min(minimum, sample.signed_mm);
    }
  }
  return minimum;
}

} // namespace

int main() {
  bool passed = true;

  const auto reference_frame = FrameId::create("fixture").value();
  const auto scan_frame = FrameId::create("scanner").value();

  const auto reference = make_patch(reference_frame, 5.0, 0.5, 0.0, 1.0);
  const auto bump = make_patch(scan_frame, 5.0, 0.5, 0.8, 1.5);
  const auto dent = make_patch(scan_frame, 5.0, 0.5, -0.8, 1.5);
  const auto config = comparison_config();

  // AC-003: a bump must yield a positive signed deviation of the injected height.
  {
    auto field = compute_deviation_field(reference.surface.view(), {}, bump.surface.view(), config);
    passed &= expect(static_cast<bool>(field), "bump deviation field must be computed");
    if (field) {
      const double peak = maximum_signed(field.value());
      passed &= expect(peak > 0.0, "bump must produce a positive signed deviation");
      passed &= expect(std::abs(peak - 0.8) <= 0.04,
                       "bump depth must match the injected height within tolerance");
      if (std::abs(peak - 0.8) > 0.04) {
        std::cerr << "  bump peak=" << peak << '\n';
      }
    }
  }

  // AC-004: a dent must yield a negative signed deviation with the correct depth.
  {
    auto field = compute_deviation_field(reference.surface.view(), {}, dent.surface.view(), config);
    passed &= expect(static_cast<bool>(field), "dent deviation field must be computed");
    if (field) {
      const double valley = minimum_signed(field.value());
      passed &= expect(valley < 0.0, "dent must produce a negative signed deviation");
      passed &= expect(std::abs(valley + 0.8) <= 0.04,
                       "dent depth must match the injected depth within tolerance");
      if (std::abs(valley + 0.8) > 0.04) {
        std::cerr << "  dent valley=" << valley << '\n';
      }
    }
  }

  // An identical scan and reference must yield near-zero signed deviation everywhere.
  {
    auto field =
        compute_deviation_field(reference.surface.view(), {}, reference.surface.view(), config);
    passed &= expect(static_cast<bool>(field), "identical surfaces must be computed");
    if (field) {
      passed &= expect(std::abs(maximum_signed(field.value())) <= 1.0e-5 &&
                           std::abs(minimum_signed(field.value())) <= 1.0e-5,
                       "identical surfaces must have zero signed deviation");
    }
  }

  // A scan displaced far from the reference must report no_neighbor rather than a false deviation.
  {
    std::vector<Vec3f> displaced_points;
    std::vector<Vec3f> displaced_normals;
    for (double x = -5.0; x <= 5.0 + 1.0e-9; x += 0.5) {
      for (double y = -5.0; y <= 5.0 + 1.0e-9; y += 0.5) {
        displaced_points.push_back({static_cast<float>(x), static_cast<float>(y), 10.0F});
        displaced_normals.push_back({0.0F, 0.0F, 1.0F});
      }
    }
    auto displaced_surface =
        OwnedSurface::create(std::move(displaced_points), std::move(displaced_normals), {},
                             std::nullopt, LengthUnit::millimeter, scan_frame)
            .value();
    auto field =
        compute_deviation_field(reference.surface.view(), {}, displaced_surface.view(), config);
    passed &= expect(static_cast<bool>(field), "displaced scan must still be computed");
    if (field) {
      std::size_t no_neighbor = 0U;
      std::size_t valid = 0U;
      for (const auto& sample : field.value().samples()) {
        no_neighbor += sample.reason == DeviationReason::no_neighbor ? 1U : 0U;
        valid += sample.reason == DeviationReason::valid ? 1U : 0U;
      }
      passed &= expect(no_neighbor > 0U && valid == 0U,
                       "a scan outside the search radius must be entirely no_neighbor");
    }
  }

  return passed ? 0 : 1;
}
