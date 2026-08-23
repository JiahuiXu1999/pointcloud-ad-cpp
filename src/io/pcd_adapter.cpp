#include "pcd_adapter.hpp"

#include "pcl_io_backend.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <utility>

namespace pointcloud_ad::io {
namespace {

[[nodiscard]] Error path_error(ErrorCode code, std::string reason, std::string_view path) {
  return Error{code,
               PipelineStage::validate,
               "invalid PCD path",
               {{"path", std::string(path)}, {"reason", std::move(reason)}}};
}

[[nodiscard]] bool has_pcd_extension(std::string_view path) {
  constexpr std::string_view extension = ".pcd";
  if (path.size() < extension.size()) {
    return false;
  }
  const auto suffix = path.substr(path.size() - extension.size());
  return std::equal(suffix.begin(), suffix.end(), extension.begin(), [](char left, char right) {
    return std::tolower(static_cast<unsigned char>(left)) ==
           std::tolower(static_cast<unsigned char>(right));
  });
}

[[nodiscard]] Result<std::string> validate_path(std::string_view path) {
  if (path.empty()) {
    return Result<std::string>::failure(
        path_error(ErrorCode::invalid_argument, "must not be empty", path));
  }
  if (path.find('\0') != std::string_view::npos) {
    return Result<std::string>::failure(
        path_error(ErrorCode::invalid_argument, "must not contain NUL", path));
  }
  if (!has_pcd_extension(path)) {
    return Result<std::string>::failure(
        path_error(ErrorCode::unsupported_format, "extension must be .pcd", path));
  }
  return Result<std::string>::success(std::string(path));
}

} // namespace

Result<OwnedSurface> read_pcd(std::string_view path_utf8, LengthUnit unit, FrameId frame) noexcept {
  auto path = validate_path(path_utf8);
  if (!path) {
    return Result<OwnedSurface>::failure(std::move(path).error());
  }
  return backends::pcl_backend::read_pcd_file(path.value(), unit, std::move(frame));
}

Result<std::size_t> write_pcd(std::string_view path_utf8, SurfaceView surface,
                              PcdEncoding encoding) noexcept {
  auto path = validate_path(path_utf8);
  if (!path) {
    return Result<std::size_t>::failure(std::move(path).error());
  }
  return backends::pcl_backend::write_pcd_file(path.value(), surface,
                                               encoding == PcdEncoding::binary);
}

} // namespace pointcloud_ad::io
