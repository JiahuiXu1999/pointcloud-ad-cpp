#include <pointcloud_ad/version.hpp>

namespace pointcloud_ad {

std::string_view version_string() noexcept {
  return POINTCLOUDAD_VERSION_STRING;
}

} // namespace pointcloud_ad
