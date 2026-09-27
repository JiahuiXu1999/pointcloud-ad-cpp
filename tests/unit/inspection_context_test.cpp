#include "performance_fixture.hpp"

#include <algorithm>
#include <future>
#include <iostream>
#include <limits>
#include <pointcloud_ad/inspection_context.hpp>
#include <tuple>

namespace {
using namespace pointcloud_ad;
bool same(const InspectionResult& a, const InspectionResult& b) {
  const auto& x = a.registration;
  const auto& y = b.registration;
  if (a.verdict != b.verdict || x.final_pose.has_value() != y.final_pose.has_value() ||
      std::tie(x.iterations, x.converged, x.valid_pairs, x.overlap_ratio, x.inlier_rmse_mm,
               x.termination_reason, x.method, x.actual_backend) !=
          std::tie(y.iterations, y.converged, y.valid_pairs, y.overlap_ratio, y.inlier_rmse_mm,
                   y.termination_reason, y.method, y.actual_backend))
    return false;
  if (x.final_pose && x.final_pose->matrix() != y.final_pose->matrix())
    return false;
  const auto& d = a.deviations;
  const auto& e = b.deviations;
  if (std::tie(d.valid_count, d.mean_signed_mm, d.rms_mm, d.max_abs_mm, d.p95_abs_mm,
               d.input_invalid, d.no_neighbor, d.normal_missing, d.normal_mismatch,
               d.reference_boundary) != std::tie(e.valid_count, e.mean_signed_mm, e.rms_mm,
                                                 e.max_abs_mm, e.p95_abs_mm, e.input_invalid,
                                                 e.no_neighbor, e.normal_missing, e.normal_mismatch,
                                                 e.reference_boundary))
    return false;
  const auto& c = a.coverage;
  const auto& f = b.coverage;
  if (std::tie(c.valid_count, c.covered_count, c.coverage_ratio, c.input_invalid, c.no_neighbor,
               c.scan_boundary) != std::tie(f.valid_count, f.covered_count, f.coverage_ratio,
                                            f.input_invalid, f.no_neighbor, f.scan_boundary))
    return false;
  if (a.regions.size() != b.regions.size() || a.diagnostics.size() != b.diagnostics.size())
    return false;
  for (std::size_t i = 0; i < a.regions.size(); ++i) {
    const auto& r = a.regions[i];
    const auto& s = b.regions[i];
    if (std::tie(r.id, r.type, r.point_count, r.centroid.x, r.centroid.y, r.centroid.z,
                 r.aabb_min.x, r.aabb_min.y, r.aabb_min.z, r.aabb_max.x, r.aabb_max.y, r.aabb_max.z,
                 r.max_abs_mm, r.mean_mm, r.rms_mm, r.p95_abs_mm, r.estimated_area_mm2,
                 r.area_is_approximate, r.severity) !=
        std::tie(s.id, s.type, s.point_count, s.centroid.x, s.centroid.y, s.centroid.z,
                 s.aabb_min.x, s.aabb_min.y, s.aabb_min.z, s.aabb_max.x, s.aabb_max.y, s.aabb_max.z,
                 s.max_abs_mm, s.mean_mm, s.rms_mm, s.p95_abs_mm, s.estimated_area_mm2,
                 s.area_is_approximate, s.severity))
      return false;
  }
  return a.provenance.run_id == b.provenance.run_id;
}
} // namespace
int main() {
  using namespace pointcloud_ad;
  bool ok = true;
  const auto rf = FrameId::create("fixture").value(), sf = FrameId::create("scanner").value();
  const auto reference = performance::patch(rf, 65, false, false),
             scan = performance::patch(sf, 65, false, false);
  std::vector<Vec3f> defect_points(scan.view().points().begin(), scan.view().points().end());
  for (auto& p : defect_points)
    p.z = static_cast<float>(0.7 * std::exp(-(p.x * p.x + p.y * p.y) / 2.0));
  const auto defect = OwnedSurface::create(defect_points,
                                           std::vector<Vec3f>(scan.view().normals().begin(),
                                                              scan.view().normals().end()),
                                           {}, {}, LengthUnit::millimeter, sf)
                          .value();
  for (auto backend : {ComputeBackend::cpu, ComputeBackend::gpu}) {
    if (!registration_backend_available(backend))
      continue;
    auto config = performance::make_config();
    config.execution.backend = backend;
    config.execution.thread_count = 4;
    std::vector<Vec3f> borrowed_points(reference.view().points().begin(),
                                       reference.view().points().end());
    auto borrowed = SurfaceView::create(borrowed_points, reference.view().normals(), {}, {},
                                        LengthUnit::millimeter, rf)
                        .value();
    auto context = InspectionContext::create(config, borrowed, scan.size());
    auto pipeline = InspectionPipeline::create(config);
    if (!context || !pipeline)
      return 1;
    for (auto& p : borrowed_points)
      p.x = std::numeric_limits<float>::quiet_NaN();
    for (auto view : {scan.view(), defect.view(), scan.view()}) {
      InspectionRequest request;
      request.run_id = view.points()[view.size() / 2].z > 0 ? "defect" : "normal";
      auto fresh = pipeline.value().run(reference.view(), view, request);
      auto reused = context.value().run(view, request);
      if (!fresh || !reused)
        return 1;
      ok &= same(fresh.value(), reused.value());
    }
    auto oversized = performance::patch(sf, 67, false, false);
    ok &= !context.value().run(oversized.view());
    InspectionRequest wrong;
    wrong.initial_pose = performance::initial(rf, sf);
    ok &= !context.value().run(scan.view(), wrong);
    auto moved = std::move(context.value());
    ok &= !context.value().run(scan.view());
    auto result = moved.run(scan.view());
    ok &= result && result.value().verdict == Verdict::pass;
    auto parallel = [&] {
      auto independent = InspectionContext::create(config, reference.view(), scan.size());
      return independent.value().run(defect.view());
    };
    auto first = std::async(std::launch::async, parallel),
         second = std::async(std::launch::async, parallel);
    auto a = first.get(), b = second.get();
    ok &= a && b && same(a.value(), b.value());
    ok &= !InspectionContext::create(config, reference.view(), 0);
  }

  // Padding is not a logical point, and masked holes must survive reference preparation.
  std::vector<Vec3f> padded_points, padded_normals;
  std::vector<std::uint8_t> mask;
  for (std::size_t row = 0; row < 65; ++row) {
    for (std::size_t col = 0; col < 65; ++col) {
      padded_points.push_back(reference.view().points()[row * 65 + col]);
      padded_normals.push_back({0, 0, 1});
      mask.push_back(row == 32 && col == 32 ? 0U : 1U);
    }
    if (row < 64) {
      padded_points.push_back({std::numeric_limits<float>::quiet_NaN(), 0, 0});
      padded_normals.push_back({});
      mask.push_back(0);
    }
  }
  auto organized_reference =
      SurfaceView::create(padded_points, padded_normals, mask, GridTopology{65, 65, 66},
                          LengthUnit::millimeter, rf)
          .value();
  auto organized_scan = SurfaceView::create(padded_points, padded_normals, mask,
                                            GridTopology{65, 65, 66}, LengthUnit::millimeter, sf)
                            .value();
  auto context = InspectionContext::create(performance::make_config(), organized_reference, 4225);
  auto pipeline = InspectionPipeline::create(performance::make_config());
  if (!context || !pipeline)
    return 1;
  const auto cached = context.value().run(organized_scan),
             fresh = pipeline.value().run(organized_reference, organized_scan);
  ok &= cached && fresh && same(cached.value(), fresh.value());
  {
    auto config = performance::make_config();
    config.execution.backend = ComputeBackend::automatic;
    config.execution.thread_count = 4;
    auto large_reference = performance::patch(rf, 256, false, false);
    auto large_scan = performance::patch(sf, 256, false, false);
    auto large_pipeline = InspectionPipeline::create(config);
    auto large_context =
        InspectionContext::create(config, large_reference.view(), large_scan.size());
    if (!large_pipeline || !large_context)
      return 1;
    const auto one = large_pipeline.value().run(large_reference.view(), large_scan.view());
    const auto reused = large_context.value().run(large_scan.view());
    const auto expected = registration_backend_available(ComputeBackend::gpu) ? ComputeBackend::gpu
                                                                              : ComputeBackend::cpu;
    ok &= one && reused && same(one.value(), reused.value()) &&
          one.value().registration.requested_backend == ComputeBackend::automatic &&
          one.value().registration.actual_backend == expected;
  }
  if (!ok)
    std::cerr << "inspection cache contract failed\n";
  return ok ? 0 : 1;
}
