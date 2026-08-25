#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <pointcloud_ad/config.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/inspection_pipeline.hpp>
#include <pointcloud_ad/inspection_result.hpp>
#include <pointcloud_ad/result.hpp>
#include <pointcloud_ad/status.hpp>
#include <pointcloud_ad/surface.hpp>
#include <pointcloud_ad/version.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include "json_serialization.hpp"
#include "output_writer.hpp"
#include "pcd_adapter.hpp"
#include "ply_adapter.hpp"

namespace {

using pointcloud_ad::Error;
using pointcloud_ad::ErrorCode;
using pointcloud_ad::FrameId;
using pointcloud_ad::InspectionConfig;
using pointcloud_ad::InspectionPipeline;
using pointcloud_ad::InspectionRequest;
using pointcloud_ad::LengthUnit;
using pointcloud_ad::OwnedSurface;
using pointcloud_ad::PipelineStage;
using pointcloud_ad::Result;
using pointcloud_ad::Verdict;
using pointcloud_ad::serialization::ManifestEntry;
using pointcloud_ad::serialization::build_manifest;
using pointcloud_ad::serialization::parse_config;
using pointcloud_ad::serialization::serialize_result;
using pointcloud_ad::serialization::sha256_hex;

constexpr int kExitOk = 0;
constexpr int kExitError = 1;
constexpr int kExitUsage = 2;

void print_help() {
  std::cout << "PointCloudAD command-line interface\n\n"
               "Usage:\n"
               "  pcad --help                         Show this help\n"
               "  pcad --version                      Show the program version\n"
               "  pcad validate-config <config.json>  Validate an inspection configuration\n"
               "  pcad inspect <config.json> <reference> <scan> [--output <dir>]\n"
               "                                      Run an inspection and write result.json and\n"
               "                                      manifest.json\n\n"
               "Point-cloud inputs are read by extension (.ply or .pcd). Inspection verdicts\n"
               "(pass/fail/indeterminate) never affect the process exit code.\n";
}

[[nodiscard]] Error cli_error(ErrorCode code, std::string message) {
  return Error{code, PipelineStage::none, std::move(message), {}};
}

[[nodiscard]] Result<std::string> read_text_file(std::string_view path) {
  std::ifstream stream(std::string(path), std::ios::in | std::ios::binary);
  if (!stream) {
    return Result<std::string>::failure(cli_error(ErrorCode::io_error, "cannot open " + std::string(path)));
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  if (stream.bad()) {
    return Result<std::string>::failure(cli_error(ErrorCode::io_error, "cannot read " + std::string(path)));
  }
  return Result<std::string>::success(buffer.str());
}

[[nodiscard]] Result<std::size_t> write_text_file(std::string_view path, std::string_view content) {
  std::ofstream stream(std::string(path), std::ios::out | std::ios::trunc | std::ios::binary);
  if (!stream) {
    return Result<std::size_t>::failure(cli_error(ErrorCode::io_error, "cannot open " + std::string(path)));
  }
  stream.write(content.data(), static_cast<std::streamsize>(content.size()));
  if (!stream) {
    return Result<std::size_t>::failure(cli_error(ErrorCode::io_error, "cannot write " + std::string(path)));
  }
  return Result<std::size_t>::success(content.size());
}

enum class PointCloudFormat : std::uint8_t { ply, pcd };

[[nodiscard]] std::optional<PointCloudFormat> format_from_path(std::string_view path) {
  const auto has_suffix = [&](std::string_view suffix) {
    return path.size() >= suffix.size() && path.substr(path.size() - suffix.size()) == suffix;
  };
  if (has_suffix(".ply")) {
    return PointCloudFormat::ply;
  }
  if (has_suffix(".pcd")) {
    return PointCloudFormat::pcd;
  }
  return std::nullopt;
}

[[nodiscard]] Result<OwnedSurface> read_surface(std::string_view path, PointCloudFormat format,
                                                LengthUnit unit, const FrameId& frame) {
  if (format == PointCloudFormat::ply) {
    return pointcloud_ad::io::read_ply(path, unit, frame);
  }
  return pointcloud_ad::io::read_pcd(path, unit, frame);
}

[[nodiscard]] std::string verdict_string(Verdict verdict) {
  switch (verdict) {
  case Verdict::pass:
    return "pass";
  case Verdict::fail:
    return "fail";
  case Verdict::indeterminate:
    return "indeterminate";
  }
  return "indeterminate";
}

void print_error(const Error& error) {
  std::cerr << "error: " << error.message << '\n';
  for (const auto& [key, value] : error.context) {
    std::cerr << "  " << key << ": " << value << '\n';
  }
}

int run_validate_config(std::string_view config_path) {
  auto text = read_text_file(config_path);
  if (!text) {
    print_error(text.error());
    return kExitError;
  }
  auto parsed = parse_config(text.value());
  if (!parsed) {
    print_error(parsed.error());
    return kExitError;
  }
  auto validated = pointcloud_ad::validate_config(parsed.value());
  if (!validated) {
    print_error(validated.error());
    return kExitError;
  }
  std::cout << "config valid\n";
  return kExitOk;
}

int run_inspect(std::string_view config_path, std::string_view reference_path,
                std::string_view scan_path, std::string_view output_dir) {
  auto text = read_text_file(config_path);
  if (!text) {
    print_error(text.error());
    return kExitError;
  }
  auto parsed = parse_config(text.value());
  if (!parsed) {
    print_error(parsed.error());
    return kExitError;
  }
  InspectionConfig config = std::move(parsed).value();
  InspectionConfig for_pipeline = config;

  auto validated = pointcloud_ad::validate_config(std::move(config));
  if (!validated) {
    print_error(validated.error());
    return kExitError;
  }

  const auto reference_format = format_from_path(reference_path);
  const auto scan_format = format_from_path(scan_path);
  if (!reference_format || !scan_format) {
    std::cerr << "error: point-cloud inputs must use a .ply or .pcd extension\n";
    return kExitUsage;
  }

  auto reference = read_surface(reference_path, *reference_format,
                                validated.value().input().reference_unit,
                                validated.value().input().reference_frame);
  if (!reference) {
    print_error(reference.error());
    return kExitError;
  }
  auto scan = read_surface(scan_path, *scan_format, validated.value().input().scan_unit,
                           validated.value().input().scan_frame);
  if (!scan) {
    print_error(scan.error());
    return kExitError;
  }

  auto pipeline = InspectionPipeline::create(std::move(for_pipeline));
  if (!pipeline) {
    print_error(pipeline.error());
    return kExitError;
  }

  InspectionRequest request;
  request.run_id = "cli-inspect";
  auto result = pipeline.value().run(reference.value().view(), scan.value().view());
  if (!result) {
    print_error(result.error());
    return kExitError;
  }

  auto result_json = serialize_result(result.value());
  if (!result_json) {
    print_error(result_json.error());
    return kExitError;
  }

  const std::filesystem::path out_dir =
      output_dir.empty() ? std::filesystem::path(".") : std::filesystem::path(output_dir);
  const auto result_path = (out_dir / "result.json").string();
  const auto manifest_path = (out_dir / "manifest.json").string();

  auto written = write_text_file(result_path, result_json.value());
  if (!written) {
    print_error(written.error());
    return kExitError;
  }

  const std::string generator = "pcad " + std::string(pointcloud_ad::version_string());
  const std::vector<ManifestEntry> entries{{"result.json", sha256_hex(result_json.value())}};
  auto manifest = build_manifest("1.0", generator, entries);
  if (!manifest) {
    print_error(manifest.error());
    return kExitError;
  }
  auto manifest_written = write_text_file(manifest_path, manifest.value());
  if (!manifest_written) {
    print_error(manifest_written.error());
    return kExitError;
  }

  std::cout << "verdict: " << verdict_string(result.value().verdict) << '\n';
  std::cout << "wrote: " << result_path << '\n';
  std::cout << "wrote: " << manifest_path << '\n';
  return kExitOk;
}

} // namespace

int main(int argc, char* argv[]) {
  if (argc == 1) {
    print_help();
    return kExitOk;
  }

  const std::string_view command{argv[1]};
  if (command == "--help" || command == "-h") {
    print_help();
    return kExitOk;
  }
  if (command == "--version") {
    std::cout << "pcad " << pointcloud_ad::version_string() << '\n';
    return kExitOk;
  }
  if (command == "validate-config") {
    if (argc != 3) {
      std::cerr << "usage: pcad validate-config <config.json>\n";
      return kExitUsage;
    }
    return run_validate_config(argv[2]);
  }
  if (command == "inspect") {
    if (argc < 5) {
      std::cerr << "usage: pcad inspect <config.json> <reference> <scan> [--output <dir>]\n";
      return kExitUsage;
    }
    std::string_view output_dir;
    if (argc == 7 && std::string_view(argv[5]) == "--output") {
      output_dir = argv[6];
    } else if (argc > 5) {
      std::cerr << "unexpected argument: " << argv[5] << "\n";
      return kExitUsage;
    }
    return run_inspect(argv[2], argv[3], argv[4], output_dir);
  }

  std::cerr << "Unknown argument: " << command << "\nUse --help for usage.\n";
  return kExitUsage;
}
