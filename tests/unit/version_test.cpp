#include <iostream>
#include <pointcloud_ad/version.hpp>
#include <string_view>

namespace {

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

} // namespace

int main() {
  constexpr auto current = pointcloud_ad::version();
  bool passed = true;

  passed &= expect(current.major == 0, "major version must be 0");
  passed &= expect(current.minor == 1, "minor version must be 1");
  passed &= expect(current.patch == 0, "patch version must be 0");
  passed &= expect(current.prerelease == "dev", "prerelease must be dev");
  passed &= expect(pointcloud_ad::version_string() == "0.1.0-dev",
                   "printable version must match the semantic version");

  return passed ? 0 : 1;
}
