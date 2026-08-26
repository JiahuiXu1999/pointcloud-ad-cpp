#pragma once

#include <pointcloud_ad/export.hpp>
#include <string_view>

namespace pointcloud_ad {

struct Version final {
  int major;
  int minor;
  int patch;
  std::string_view prerelease;
};

[[nodiscard]] constexpr Version version() noexcept {
  return Version{0, 1, 0, {}};
}

[[nodiscard]] POINTCLOUD_AD_EXPORT std::string_view version_string() noexcept;

} // namespace pointcloud_ad
