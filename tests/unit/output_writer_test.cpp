#include "output_writer.hpp"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <pointcloud_ad/geometry.hpp>
#include <pointcloud_ad/surface.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

using pointcloud_ad::FrameId;
using pointcloud_ad::LengthUnit;
using pointcloud_ad::OwnedSurface;
using pointcloud_ad::Vec3f;
using pointcloud_ad::serialization::ManifestEntry;
using pointcloud_ad::serialization::build_manifest;
using pointcloud_ad::serialization::sha256_hex;
using pointcloud_ad::serialization::write_attribute_ply;

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

[[nodiscard]] OwnedSurface make_surface() {
  std::vector<Vec3f> points{{0.0F, 0.0F, 0.0F},
                            {1.0F, 0.0F, 0.0F},
                            {0.0F, 1.0F, 0.0F},
                            {1.0F, 1.0F, 0.0F}};
  return OwnedSurface::create(std::move(points), {}, {}, std::nullopt, LengthUnit::millimeter,
                              FrameId::create("fixture").value())
      .value();
}

[[nodiscard]] std::string read_file(std::string_view path) {
  std::ifstream stream(std::string(path), std::ios::in | std::ios::binary);
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

} // namespace

int main() {
  bool passed = true;

  // SHA-256 matches published test vectors.
  {
    passed &= expect(sha256_hex("") ==
                         "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
                     "SHA-256 of empty string");
    passed &= expect(sha256_hex("abc") ==
                         "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
                     "SHA-256 of \"abc\"");
    passed &= expect(sha256_hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
                         "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1",
                     "SHA-256 of two-block message");
  }

  // write_attribute_ply emits an ASCII PLY with the declared scalar/reason/region fields.
  {
    const auto surface = make_surface();
    const std::vector<double> scalar{0.1, 0.2, 0.3, 0.4};
    const std::vector<std::uint8_t> reason{0U, 1U, 2U, 3U};
    const std::vector<std::int32_t> region{0, -1, 1, 2};
    const std::string path = "output_writer_test.ply";
    auto written = write_attribute_ply(path, surface.view(), scalar, reason, region);
    passed &= expect(static_cast<bool>(written), "attribute PLY must write");
    if (written) {
      passed &= expect(written.value() == 4U, "attribute PLY must report 4 vertices");
      const std::string content = read_file(path);
      passed &= expect(content.find("format ascii 1.0") != std::string::npos,
                       "attribute PLY must be ASCII");
      passed &= expect(content.find("element vertex 4") != std::string::npos,
                       "attribute PLY must declare 4 vertices");
      passed &= expect(content.find("property float scalar") != std::string::npos,
                       "attribute PLY must declare the scalar property");
      passed &= expect(content.find("property uchar reason") != std::string::npos,
                       "attribute PLY must declare the reason property");
      passed &= expect(content.find("property int region_id") != std::string::npos,
                       "attribute PLY must declare the region_id property");
      passed &= expect(content.find("0.300000 2 1") != std::string::npos,
                       "attribute PLY must write scalar/reason/region values");
    }
    std::remove(path.c_str());
  }

  // A size mismatch must fail without writing.
  {
    const auto surface = make_surface();
    const std::vector<double> scalar{0.1};
    const std::vector<std::uint8_t> reason{0U, 1U, 2U, 3U};
    const std::vector<std::int32_t> region{0, 1, 2, 3};
    auto written = write_attribute_ply("unused.ply", surface.view(), scalar, reason, region);
    passed &= expect(!written, "mismatched attribute span must fail");
  }

  // build_manifest emits a versioned document listing each file with its digest.
  {
    std::vector<ManifestEntry> entries{{"result.json", "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"}};
    auto manifest = build_manifest("1.0", "pcad", entries);
    passed &= expect(static_cast<bool>(manifest), "manifest must build");
    if (manifest) {
      const auto document = nlohmann::json::parse(manifest.value());
      passed &= expect(document["schema_version"].get<std::string>() == "1.0",
                       "manifest schema_version must be present");
      passed &= expect(document["generator"].get<std::string>() == "pcad",
                       "manifest generator must be present");
      passed &= expect(document["files"].is_array() && document["files"].size() == 1U,
                       "manifest must list exactly one file");
      passed &= expect(document["files"][0]["path"].get<std::string>() == "result.json",
                       "manifest must record the file path");
      passed &= expect(document["files"][0]["sha256"].get<std::string>().size() == 64U,
                       "manifest must record a 64-character SHA-256 digest");
    }
  }

  return passed ? 0 : 1;
}
