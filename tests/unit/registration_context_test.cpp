#include "backend_policy.hpp"

#include <array>
#include <cmath>
#include <future>
#include <iostream>
#include <limits>
#include <pointcloud_ad/registration_engine.hpp>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
using namespace pointcloud_ad;
constexpr std::array<double, 16> kIdentity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

OwnedSurface patch(const FrameId& frame, bool scan, bool normals) {
  std::vector<Vec3f> points;
  std::vector<Vec3f> directions;
  const double angle = 0.015;
  const double c = std::cos(angle), s = std::sin(angle);
  for (int x = -8; x <= 8; ++x) {
    for (int y = -8; y <= 8; ++y) {
      const double px = 0.5 * x, py = 0.5 * y;
      const double pz = 0.08 * px * px + 0.13 * py * py + 0.003 * px * py * py;
      double nx = -0.16 * px - 0.003 * py * py;
      double ny = -0.26 * py - 0.006 * px * py;
      const double length = std::sqrt(nx * nx + ny * ny + 1.0);
      nx /= length;
      ny /= length;
      if (scan) {
        points.push_back({static_cast<float>(c * (px - 0.12) + s * (py + 0.08)),
                          static_cast<float>(-s * (px - 0.12) + c * (py + 0.08)),
                          static_cast<float>(pz - 0.10)});
        if (normals) {
          directions.push_back({static_cast<float>(c * nx + s * ny),
                                static_cast<float>(-s * nx + c * ny),
                                static_cast<float>(1.0 / length)});
        }
      } else {
        points.push_back({static_cast<float>(px), static_cast<float>(py), static_cast<float>(pz)});
        if (normals) {
          directions.push_back(
              {static_cast<float>(nx), static_cast<float>(ny), static_cast<float>(1.0 / length)});
        }
      }
    }
  }
  std::vector<std::uint8_t> mask(points.size(), 1U);
  points.push_back({std::numeric_limits<float>::quiet_NaN(), 0, 0});
  mask.push_back(0U);
  if (normals) {
    directions.push_back({0, 0, 0});
  }
  return OwnedSurface::create(std::move(points), std::move(directions), std::move(mask),
                              std::nullopt, LengthUnit::millimeter, frame)
      .value();
}

RegistrationParameters parameters(RegistrationMethod method) {
  RegistrationParameters p;
  p.max_iterations = 100;
  p.max_correspondence_distance_mm = 1.0;
  p.huber_delta_mm = 0.5;
  p.translation_epsilon_mm = 1.0e-8;
  p.rotation_epsilon_rad = 1.0e-9;
  p.residual_epsilon_mm = 1.0e-10;
  p.method = method;
  p.thread_count = 4;
  return p;
}
} // namespace

bool identical(const RegistrationMetrics& lhs, const RegistrationMetrics& rhs) {
  return lhs.final_transform().matrix() == rhs.final_transform().matrix() &&
         lhs.convergence() == rhs.convergence() && lhs.iterations() == rhs.iterations() &&
         lhs.valid_pairs() == rhs.valid_pairs() && lhs.fitness() == rhs.fitness() &&
         lhs.inlier_rmse_mm() == rhs.inlier_rmse_mm() &&
         lhs.translation_delta_mm() == rhs.translation_delta_mm() &&
         lhs.rotation_delta_deg() == rhs.rotation_delta_deg();
}

int main() {
  static_assert(!std::is_copy_constructible_v<RegistrationContext>);
  static_assert(std::is_nothrow_move_constructible_v<RegistrationContext>);
  static_assert(std::is_nothrow_move_assignable_v<RegistrationContext>);
  bool passed = true;
  using registration::select_backend;
  passed &= expect(
      select_backend(ComputeBackend::automatic, 65535, 65536, true) == ComputeBackend::cpu &&
          select_backend(ComputeBackend::automatic, 65536, 65535, true) == ComputeBackend::cpu &&
          select_backend(ComputeBackend::automatic, 65536, 65536, true) == ComputeBackend::gpu &&
          select_backend(ComputeBackend::automatic, 65536, 65536, false) == ComputeBackend::cpu &&
          select_backend(ComputeBackend::cpu, 1000000, 1000000, true) == ComputeBackend::cpu &&
          select_backend(ComputeBackend::gpu, 1, 1, false) == ComputeBackend::gpu,
      "automatic boundaries and explicit overrides");

  const auto reference_frame = FrameId::create("reference").value();
  const auto scan_frame = FrameId::create("scan").value();
  const auto initial = RigidTransform::create(kIdentity, scan_frame, reference_frame).value();
  const auto wrong = RigidTransform::create(kIdentity, reference_frame, scan_frame).value();
  const auto reference = patch(reference_frame, false, true);
  const auto scan = patch(scan_frame, true, true);
  const auto second = patch(scan_frame, false, true);
#ifdef POINTCLOUDAD_TEST_CUDA
  passed &= expect(registration_backend_available(ComputeBackend::gpu), "GPU must be available");
#endif
  for (auto backend : {ComputeBackend::cpu, ComputeBackend::automatic, ComputeBackend::gpu}) {
    if (!registration_backend_available(backend)) {
      passed &= expect(!RegistrationContext::create(reference.view(),
                                                    parameters(RegistrationMethod::point_to_point),
                                                    scan.size(), backend),
                       "unavailable GPU rejects context");
      continue;
    }
    for (auto method : {RegistrationMethod::point_to_plane, RegistrationMethod::point_to_point,
                        RegistrationMethod::gicp}) {
      auto options = parameters(method);
      auto fresh = register_surfaces(reference.view(), scan.view(), initial, options, backend);
      if (!fresh) {
        passed = false;
        continue;
      }
      // Input snapshot is independent of caller mutations and of storage/frame destruction.
      auto created = [&] {
        std::vector<Vec3f> points(reference.view().points().begin(),
                                  reference.view().points().end());
        std::vector<Vec3f> normals(reference.view().normals().begin(),
                                   reference.view().normals().end());
        std::vector<std::uint8_t> valid(reference.view().valid().begin(),
                                        reference.view().valid().end());
        auto frame = reference_frame;
        auto view =
            SurfaceView::create(points, normals, valid, {}, LengthUnit::millimeter, frame).value();
        auto result = RegistrationContext::create(view, options, scan.size(), backend);
        points[0].x = 10000;
        normals[0] = {};
        valid[0] = 0;
        frame = scan_frame;
        return result;
      }();
      if (!created) {
        std::cerr << created.error().message << '\n';
        passed = false;
        continue;
      }
      auto context = std::move(created).value();
      passed &=
          expect(!created.value().align(scan.view(), initial), "moved-from context returns error");
      options.max_iterations = 1;
      options.covariance_epsilon = 0.7;
      for (int run = 0; run < 3; ++run) {
        auto actual = context.align(scan.view(), initial);
        passed &= expect(actual && identical(actual.value(), fresh.value()),
                         "snapshot and parameters survive; reused metrics exactly equal fresh");
        auto other = context.align(second.view(), initial);
        auto other_fresh = register_surfaces(reference.view(), second.view(), initial,
                                             parameters(method), backend);
        passed &= expect(other && other_fresh && identical(other.value(), other_fresh.value()),
                         "different scan does not reuse old scan covariance or pose");
      }
      passed &= expect(!context.align(scan.view(), wrong), "wrong frame rejected");
      auto meters = SurfaceView::create(scan.view().points(), scan.view().normals(),
                                        scan.view().valid(), {}, LengthUnit::meter, scan_frame)
                        .value();
      passed &= expect(!context.align(meters, initial), "wrong units rejected");
      std::vector<Vec3f> too_many(scan.size() + 1, Vec3f{1, 2, 3});
      auto oversized =
          SurfaceView::create(too_many, {}, {}, {}, LengthUnit::millimeter, scan_frame).value();
      passed &= expect(!context.align(oversized, initial), "logical capacity exceeded");
      std::vector<Vec3f> mutable_scan(scan.view().points().begin(),
                                      scan.view().points().begin() + 100);
      auto short_scan =
          SurfaceView::create(mutable_scan, {}, {}, {}, LengthUnit::millimeter, scan_frame).value();
      auto short_fresh =
          register_surfaces(reference.view(), short_scan, initial, parameters(method), backend);
      auto short_reused = context.align(short_scan, initial);
      passed &= expect(short_fresh && short_reused &&
                           identical(short_fresh.value(), short_reused.value()),
                       "smaller scans reuse reserved buffers");
      mutable_scan[0].x = std::numeric_limits<float>::quiet_NaN();
      passed &= expect(!context.align(short_scan, initial), "mutated borrowed scan revalidated");
      auto recovered = context.align(scan.view(), initial);
      passed &= expect(recovered && identical(recovered.value(), fresh.value()),
                       "rejected inputs do not invalidate prepared reference");
      auto destination_result =
          RegistrationContext::create(reference.view(), parameters(method), scan.size(), backend);
      if (!destination_result) {
        passed = false;
        continue;
      }
      auto destination = std::move(destination_result).value();
      destination = std::move(context);
      passed &= expect(!context.align(scan.view(), initial), "move assignment empties source");
      auto assigned = destination.align(scan.view(), initial);
      passed &= expect(assigned && identical(assigned.value(), fresh.value()),
                       "move assignment transfers resources");
      auto independent =
          RegistrationContext::create(reference.view(), parameters(method), scan.size(), backend);
      if (!independent) {
        passed = false;
        continue;
      }
      auto first_job =
          std::async(std::launch::async, [&] { return destination.align(scan.view(), initial); });
      auto second_job = std::async(
          std::launch::async, [&] { return independent.value().align(second.view(), initial); });
      auto first_result = first_job.get();
      auto second_result = second_job.get();
      auto second_expected =
          register_surfaces(reference.view(), second.view(), initial, parameters(method), backend);
      passed &= expect(first_result && second_result && second_expected &&
                           identical(first_result.value(), fresh.value()) &&
                           identical(second_result.value(), second_expected.value()),
                       "independent concurrent contexts match both serial results");
      if (method == RegistrationMethod::gicp) {
        auto changed = parameters(method);
        changed.covariance_neighbors = 20;
        changed.covariance_epsilon = 0.1;
        auto alternate =
            RegistrationContext::create(reference.view(), changed, scan.size(), backend);
        auto expected = register_surfaces(reference.view(), scan.view(), initial, changed, backend);
        if (!alternate || !expected) {
          passed = false;
          continue;
        }
        auto actual = alternate.value().align(scan.view(), initial);
        passed &= expect(actual && identical(actual.value(), expected.value()),
                         "new covariance settings build an independent reference cache");
      }
    }
  }
  // Organized storage contains padding that is neither a logical point nor GPU capacity.
  std::vector<Vec3f> padded_points, padded_normals;
  for (std::size_t row = 0; row < 17U; ++row) {
    for (std::size_t col = 0; col < 17U; ++col) {
      padded_points.push_back(reference.view().points()[row * 17U + col]);
      padded_normals.push_back(reference.view().normals()[row * 17U + col]);
    }
    if (row < 16U) {
      padded_points.push_back({std::numeric_limits<float>::quiet_NaN(), 0, 0});
      padded_normals.push_back({});
    }
  }
  auto organized = SurfaceView::create(padded_points, padded_normals, {}, GridTopology{17, 17, 18},
                                       LengthUnit::millimeter, reference_frame)
                       .value();
  auto organized_scan =
      SurfaceView::create(padded_points, padded_normals, {}, GridTopology{17, 17, 18},
                          LengthUnit::millimeter, scan_frame)
          .value();
  for (auto backend : {ComputeBackend::cpu, ComputeBackend::gpu}) {
    if (!registration_backend_available(backend)) {
      continue;
    }
    for (auto method : {RegistrationMethod::point_to_plane, RegistrationMethod::point_to_point,
                        RegistrationMethod::gicp}) {
      auto context = RegistrationContext::create(organized, parameters(method), 289U, backend);
      auto fresh =
          register_surfaces(organized, organized_scan, initial, parameters(method), backend);
      if (!context || !fresh) {
        passed = false;
        continue;
      }
      auto actual = context.value().align(organized_scan, initial);
      passed &= expect(actual && identical(actual.value(), fresh.value()) &&
                           actual.value().valid_pairs() == 289U,
                       "organized padding ignored; capacity bounds logical size");
    }
  }
  std::vector<std::uint8_t> invalid_mask(reference.view().storage_size(), 0U);
  auto empty_valid = SurfaceView::create(reference.view().points(), reference.view().normals(),
                                         invalid_mask, {}, LengthUnit::millimeter, reference_frame)
                         .value();
  passed &= expect(!RegistrationContext::create(
                       empty_valid, parameters(RegistrationMethod::point_to_point), scan.size()),
                   "all-invalid reference rejected");
  auto options = parameters(RegistrationMethod::point_to_point);
  passed &=
      expect(!RegistrationContext::create(reference.view(), options, 0), "zero capacity rejected");
  passed &= expect(!RegistrationContext::create(
                       reference.view(), options,
                       static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()) + 1U),
                   "oversized capacity rejected before allocation");
  passed &= expect(!RegistrationContext::create(reference.view(), options, scan.size(),
                                                static_cast<ComputeBackend>(255)),
                   "unknown backend rejected");
  options.max_iterations = 0;
  passed &= expect(!RegistrationContext::create(reference.view(), options, scan.size()),
                   "invalid parameters rejected");
  const auto bare = patch(reference_frame, false, false);
  passed &= expect(!RegistrationContext::create(
                       bare.view(), parameters(RegistrationMethod::point_to_plane), scan.size()),
                   "missing plane normals rejected at prepare");
  auto meters =
      SurfaceView::create(reference.view().points(), reference.view().normals(),
                          reference.view().valid(), {}, LengthUnit::meter, reference_frame)
          .value();
  passed &= expect(!RegistrationContext::create(
                       meters, parameters(RegistrationMethod::point_to_point), scan.size()),
                   "unnormalized reference rejected");
  {
    std::vector<Vec3f> points, normals;
    for (int y = 0; y < 256; ++y)
      for (int x = 0; x < 256; ++x) {
        points.push_back({static_cast<float>(x) * 0.25F, static_cast<float>(y) * 0.25F, 0});
        normals.push_back({0, 0, 1});
      }
    const auto frame = FrameId::create("large").value();
    auto surface = OwnedSurface::create(std::move(points), std::move(normals), {}, {},
                                        LengthUnit::millimeter, frame)
                       .value();
    auto context =
        RegistrationContext::create(surface.view(), parameters(RegistrationMethod::point_to_plane),
                                    surface.size(), ComputeBackend::automatic);
    passed &= expect(static_cast<bool>(context), "large automatic context");
    if (context) {
      const auto expected = registration_backend_available(ComputeBackend::gpu)
                                ? ComputeBackend::gpu
                                : ComputeBackend::cpu;
      passed &= expect(context.value().backend() == expected, "observable automatic backend");
      const auto identity = RigidTransform::create(kIdentity, frame, frame).value();
      const auto solved = context.value().align(surface.view(), identity);
      passed &= expect(solved && solved.value().valid_pairs() == surface.size() &&
                           solved.value().inlier_rmse_mm() == 0,
                       "large automatic registration geometry");
      auto moved = std::move(context.value());
      passed &= expect(context.value().backend() == ComputeBackend::automatic &&
                           moved.backend() == expected,
                       "moved context selection sentinel");
    }
  }
  return passed ? 0 : 1;
}
