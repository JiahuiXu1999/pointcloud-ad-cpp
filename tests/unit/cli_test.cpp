#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/surface.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "json_serialization.hpp"
#include "ply_adapter.hpp"

#ifndef PCAD_EXE
#define PCAD_EXE "pcad"
#endif

namespace {

using pointcloud_ad::FrameId;
using pointcloud_ad::InspectionConfig;
using pointcloud_ad::LengthUnit;
using pointcloud_ad::OwnedSurface;
using pointcloud_ad::Vec3f;
using pointcloud_ad::io::write_ply;
using pointcloud_ad::serialization::serialize_config;

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

[[nodiscard]] std::string quote(std::string_view value) {
  return "\"" + std::string(value) + "\"";
}

int run_command(const std::string& command) {
  return std::system(command.c_str());
}

[[nodiscard]] std::string read_file(std::string_view path) {
  std::ifstream stream(std::string(path), std::ios::in | std::ios::binary);
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

[[nodiscard]] InspectionConfig make_config() {
  InspectionConfig config;
  config.schema_version = "1.0";
  config.profile = "synthetic_demo";
  config.input.reference_unit = LengthUnit::millimeter;
  config.input.scan_unit = LengthUnit::millimeter;
  config.input.reference_frame = FrameId::create("fixture").value();
  config.input.scan_frame = FrameId::create("scanner").value();
  config.preprocess.voxel_size_mm = 0.2;
  config.preprocess.normal_radius_mm = 1.0;
  config.preprocess.normal_min_neighbors = 12;
  config.preprocess.boundary_radius_mm = 0.8;
  config.registration.max_iterations = 60;
  config.registration.max_correspondence_distance_mm = 1.0;
  config.registration.huber_delta_mm = 0.3;
  config.registration.translation_epsilon_mm = 0.0001;
  config.registration.rotation_epsilon_rad = 0.00001;
  config.registration.residual_epsilon_mm = 0.00001;
  config.registration_gate.min_overlap_ratio = 0.7;
  config.registration_gate.max_inlier_rmse_mm = 0.2;
  config.registration_gate.min_valid_pairs = 500;
  config.registration_gate.max_translation_from_initial_mm = 5.0;
  config.registration_gate.max_rotation_from_initial_deg = 5.0;
  config.comparison.max_search_distance_mm = 0.8;
  config.comparison.max_normal_angle_deg = 35.0;
  config.comparison.boundary_exclusion_mm = 0.6;
  config.comparison.min_valid_coverage_ratio = 0.75;
  config.detection.positive_threshold_mm = 0.25;
  config.detection.negative_threshold_mm = -0.25;
  config.detection.cluster_tolerance_mm = 0.6;
  config.detection.min_cluster_points = 20;
  config.detection.measurement_error_budget_mm = 0.0;
  config.execution.deterministic = true;
  config.execution.thread_count = 1;
  config.execution.random_seed = 5489;
  return config;
}

[[nodiscard]] OwnedSurface make_patch(const FrameId& frame) {
  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  for (double x = -6.0; x <= 6.0 + 1.0e-9; x += 0.5) {
    for (double y = -6.0; y <= 6.0 + 1.0e-9; y += 0.5) {
      points.push_back({static_cast<float>(x), static_cast<float>(y), 0.0F});
      normals.push_back({0.0F, 0.0F, 1.0F});
    }
  }
  return OwnedSurface::create(std::move(points), std::move(normals), {}, std::nullopt,
                              LengthUnit::millimeter, frame)
      .value();
}

bool write_file(std::string_view path, std::string_view content) {
  std::ofstream stream(std::string(path), std::ios::out | std::ios::trunc | std::ios::binary);
  if (!stream) {
    return false;
  }
  stream.write(content.data(), static_cast<std::streamsize>(content.size()));
  return static_cast<bool>(stream);
}

} // namespace

int main() {
  bool passed = true;

  const auto reference_frame = FrameId::create("fixture").value();
  const auto scan_frame = FrameId::create("scanner").value();

  // Write fixtures: a valid config, an invalid config, and two identical planar clouds.
  const std::string config_path = "cli_test_config.json";
  const std::string invalid_config_path = "cli_test_invalid_config.json";
  const std::string reference_path = "cli_test_reference.ply";
  const std::string scan_path = "cli_test_scan.ply";
  const std::string output_dir = "cli_test_output";

  const auto config = make_config();
  auto config_json = serialize_config(config);
  passed &= expect(static_cast<bool>(config_json), "config must serialize");
  passed &= expect(write_file(config_path, config_json ? config_json.value() : ""),
                   "config fixture must write");
  passed &= expect(write_file(invalid_config_path, "{\"schema_version\":\"9.9\"}"),
                   "invalid config fixture must write");

  const auto reference = make_patch(reference_frame);
  const auto scan = make_patch(scan_frame);
  auto ref_written = write_ply(reference_path, reference.view());
  auto scan_written = write_ply(scan_path, scan.view());
  passed &= expect(static_cast<bool>(ref_written), "reference fixture must write");
  passed &= expect(static_cast<bool>(scan_written), "scan fixture must write");
  std::filesystem::create_directories(output_dir);

  // PCAD_EXE already carries its own quoting from the CMake compile definition.
  const std::string exe = PCAD_EXE;

  // --version
  {
    const int code = run_command(exe + " --version");
    passed &= expect(code == 0, "--version must exit 0");
  }

  // validate-config on a valid document.
  {
    const int code = run_command(exe + " validate-config " + quote(config_path));
    passed &= expect(code == 0, "validate-config on a valid config must exit 0");
  }

  // validate-config on an invalid document.
  {
    const int code = run_command(exe + " validate-config " + quote(invalid_config_path));
    passed &= expect(code != 0, "validate-config on an invalid config must fail");
  }

  // inspect completes the vertical slice and writes result.json + manifest.json.
  {
    const int code = run_command(exe + " inspect " + quote(config_path) + " " +
                                 quote(reference_path) + " " + quote(scan_path) + " --output " +
                                 quote(output_dir));
    passed &= expect(code == 0, "inspect must exit 0");

    const std::string result_json = read_file(output_dir + "/result.json");
    passed &= expect(!result_json.empty(), "inspect must write result.json");
    if (!result_json.empty()) {
      const auto document = nlohmann::json::parse(result_json);
      passed &= expect(document.contains("verdict") && document.contains("provenance"),
                       "result.json must carry verdict and provenance");
      passed &= expect(document["verdict"].get<std::string>() == "pass",
                       "identical planar clouds must yield a pass verdict");
    }

    const std::string manifest_json = read_file(output_dir + "/manifest.json");
    passed &= expect(!manifest_json.empty(), "inspect must write manifest.json");
    if (!manifest_json.empty()) {
      const auto document = nlohmann::json::parse(manifest_json);
      passed &= expect(document["files"].is_array() && document["files"].size() == 1U,
                       "manifest must list the result file");
      passed &= expect(document["files"][0]["path"].get<std::string>() == "result.json",
                       "manifest must record result.json");
      passed &= expect(document["files"][0]["sha256"].get<std::string>().size() == 64U,
                       "manifest must record a SHA-256 digest");
    }
  }

  // Unknown arguments remain a usage error.
  {
    const int code = run_command(exe + " --bogus");
    passed &= expect(code == 2, "unknown arguments must exit 2");
  }

  std::remove(config_path.c_str());
  std::remove(invalid_config_path.c_str());
  std::remove(reference_path.c_str());
  std::remove(scan_path.c_str());
  std::remove((output_dir + "/result.json").c_str());
  std::remove((output_dir + "/manifest.json").c_str());

  return passed ? 0 : 1;
}
