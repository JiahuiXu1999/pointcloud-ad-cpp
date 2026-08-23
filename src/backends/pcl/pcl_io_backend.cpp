#include "pcl_io_backend.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <map>
#include <optional>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_ad::backends::pcl_backend {
namespace {

[[nodiscard]] Error io_error(ErrorCode code, std::string message, std::string_view path,
                             std::string reason = {}) {
  std::map<std::string, std::string> context{{"path", std::string(path)}};
  if (!reason.empty()) {
    context.emplace("reason", std::move(reason));
  }
  return Error{code, PipelineStage::none, std::move(message), std::move(context)};
}

[[nodiscard]] const pcl::PCLPointField* find_field(const pcl::PCLPointCloud2& cloud,
                                                   std::string_view name) noexcept {
  for (const auto& field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

[[nodiscard]] bool is_float_scalar(const pcl::PCLPointField* field) noexcept {
  return field != nullptr && field->count == 1U && field->datatype == pcl::PCLPointField::FLOAT32;
}

[[nodiscard]] std::size_t storage_offset(const pcl::PCLPointCloud2& cloud,
                                         std::size_t logical_index) noexcept {
  const auto width = static_cast<std::size_t>(cloud.width);
  const auto row = logical_index / width;
  const auto column = logical_index % width;
  return row * static_cast<std::size_t>(cloud.row_step) +
         column * static_cast<std::size_t>(cloud.point_step);
}

template <typename Value>
[[nodiscard]] Value read_field(const pcl::PCLPointCloud2& cloud, std::size_t logical_index,
                               const pcl::PCLPointField& field) noexcept {
  Value value{};
  const auto offset = storage_offset(cloud, logical_index) + field.offset;
  std::memcpy(&value, cloud.data.data() + offset, sizeof(Value));
  return value;
}

template <typename Value>
void write_field(pcl::PCLPointCloud2& cloud, std::size_t logical_index,
                 const pcl::PCLPointField& field, Value value) noexcept {
  const auto offset = storage_offset(cloud, logical_index) + field.offset;
  std::memcpy(cloud.data.data() + offset, &value, sizeof(Value));
}

void add_field(pcl::PCLPointCloud2& cloud, std::string name, std::uint32_t offset,
               std::uint8_t datatype) {
  cloud.fields.push_back(pcl::PCLPointField{std::move(name), offset, datatype, 1U});
}

template <typename Function> void for_each_logical_index(SurfaceView surface, Function&& function) {
  if (!surface.grid()) {
    for (std::size_t index = 0; index < surface.storage_size(); ++index) {
      function(index);
    }
    return;
  }
  const auto& grid = *surface.grid();
  for (std::size_t row = 0; row < grid.height; ++row) {
    for (std::size_t column = 0; column < grid.width; ++column) {
      function(row * static_cast<std::size_t>(grid.row_stride) + column);
    }
  }
}

[[nodiscard]] Result<std::size_t> validate_fields(const pcl::PCLPointCloud2& cloud,
                                                  std::string_view path, std::string_view format) {
  if (!is_float_scalar(find_field(cloud, "x")) || !is_float_scalar(find_field(cloud, "y")) ||
      !is_float_scalar(find_field(cloud, "z"))) {
    return Result<std::size_t>::failure(io_error(
        ErrorCode::unsupported_format, std::string(format) + " requires float32 XYZ fields", path));
  }
  const bool normal_x = is_float_scalar(find_field(cloud, "normal_x"));
  const bool normal_y = is_float_scalar(find_field(cloud, "normal_y"));
  const bool normal_z = is_float_scalar(find_field(cloud, "normal_z"));
  if ((normal_x || normal_y || normal_z) && !(normal_x && normal_y && normal_z)) {
    return Result<std::size_t>::failure(
        io_error(ErrorCode::unsupported_format,
                 std::string(format) + " normal fields must be complete float32 triples", path));
  }
  const auto* valid = find_field(cloud, "valid");
  if (valid != nullptr && (valid->count != 1U || valid->datatype != pcl::PCLPointField::UINT8)) {
    return Result<std::size_t>::failure(io_error(
        ErrorCode::unsupported_format, std::string(format) + " valid field must be uint8", path));
  }
  const auto count = static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
  const auto minimum_row_step = static_cast<std::size_t>(cloud.width) * cloud.point_step;
  const auto minimum_storage =
      cloud.height == 0U
          ? 0U
          : (static_cast<std::size_t>(cloud.height) - 1U) * cloud.row_step + minimum_row_step;
  if (count == 0U || cloud.point_step == 0U || cloud.row_step < minimum_row_step ||
      cloud.data.size() < minimum_storage) {
    return Result<std::size_t>::failure(
        io_error(ErrorCode::invalid_input,
                 std::string(format) + " contains inconsistent point storage", path));
  }
  return Result<std::size_t>::success(count);
}

[[nodiscard]] Result<OwnedSurface> decode_cloud(const pcl::PCLPointCloud2& cloud, LengthUnit unit,
                                                FrameId frame, std::string_view path,
                                                std::string_view format) {
  auto validated = validate_fields(cloud, path, format);
  if (!validated) {
    return Result<OwnedSurface>::failure(std::move(validated).error());
  }
  const auto count = validated.value();
  const auto* x = find_field(cloud, "x");
  const auto* y = find_field(cloud, "y");
  const auto* z = find_field(cloud, "z");
  const auto* normal_x = find_field(cloud, "normal_x");
  const auto* normal_y = find_field(cloud, "normal_y");
  const auto* normal_z = find_field(cloud, "normal_z");
  const auto* valid = find_field(cloud, "valid");
  const bool has_normals = normal_x != nullptr;
  const bool has_mask = valid != nullptr;

  std::vector<Vec3f> points;
  std::vector<Vec3f> normals;
  std::vector<std::uint8_t> mask;
  points.reserve(count);
  if (has_normals) {
    normals.reserve(count);
  }
  mask.reserve(count);
  bool derived_mask_needed = false;
  for (std::size_t index = 0; index < count; ++index) {
    const Vec3f point{read_field<float>(cloud, index, *x), read_field<float>(cloud, index, *y),
                      read_field<float>(cloud, index, *z)};
    points.push_back(point);
    bool finite = std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    if (has_normals) {
      const Vec3f normal{read_field<float>(cloud, index, *normal_x),
                         read_field<float>(cloud, index, *normal_y),
                         read_field<float>(cloud, index, *normal_z)};
      normals.push_back(normal);
      finite =
          finite && std::isfinite(normal.x) && std::isfinite(normal.y) && std::isfinite(normal.z);
    }
    if (has_mask) {
      const auto mask_value = read_field<std::uint8_t>(cloud, index, *valid);
      if (mask_value > 1U) {
        return Result<OwnedSurface>::failure(io_error(
            ErrorCode::invalid_input, std::string(format) + " valid values must be 0 or 1", path));
      }
      if (mask_value == 1U && !finite) {
        return Result<OwnedSurface>::failure(
            io_error(ErrorCode::invalid_input,
                     std::string(format) + " marks a non-finite sample as valid", path));
      }
      mask.push_back(mask_value);
    } else {
      derived_mask_needed = derived_mask_needed || !finite;
      mask.push_back(finite ? 1U : 0U);
    }
  }
  if (!has_mask && !derived_mask_needed) {
    mask.clear();
  }

  std::optional<GridTopology> grid;
  if (cloud.height > 1U) {
    grid = GridTopology{cloud.width, cloud.height, cloud.width};
  }
  return OwnedSurface::create(std::move(points), std::move(normals), std::move(mask), grid, unit,
                              std::move(frame));
}

[[nodiscard]] pcl::PCLPointCloud2 encode_cloud(SurfaceView surface) {
  pcl::PCLPointCloud2 cloud;
  const bool has_normals = !surface.normals().empty();
  add_field(cloud, "x", 0U, pcl::PCLPointField::FLOAT32);
  add_field(cloud, "y", 4U, pcl::PCLPointField::FLOAT32);
  add_field(cloud, "z", 8U, pcl::PCLPointField::FLOAT32);
  if (has_normals) {
    add_field(cloud, "normal_x", 12U, pcl::PCLPointField::FLOAT32);
    add_field(cloud, "normal_y", 16U, pcl::PCLPointField::FLOAT32);
    add_field(cloud, "normal_z", 20U, pcl::PCLPointField::FLOAT32);
    add_field(cloud, "valid", 24U, pcl::PCLPointField::UINT8);
    cloud.point_step = 25U;
  } else {
    add_field(cloud, "valid", 12U, pcl::PCLPointField::UINT8);
    cloud.point_step = 13U;
  }
  cloud.width = surface.grid() ? surface.grid()->width : static_cast<std::uint32_t>(surface.size());
  cloud.height = surface.grid() ? surface.grid()->height : 1U;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(surface.size() * cloud.point_step);
  const auto* x = find_field(cloud, "x");
  const auto* y = find_field(cloud, "y");
  const auto* z = find_field(cloud, "z");
  const auto* normal_x = find_field(cloud, "normal_x");
  const auto* normal_y = find_field(cloud, "normal_y");
  const auto* normal_z = find_field(cloud, "normal_z");
  const auto* valid = find_field(cloud, "valid");
  bool dense = true;
  std::size_t logical_index = 0U;
  for_each_logical_index(surface, [&](std::size_t index) {
    const auto point = surface.points()[index];
    write_field(cloud, logical_index, *x, point.x);
    write_field(cloud, logical_index, *y, point.y);
    write_field(cloud, logical_index, *z, point.z);
    const auto mask = surface.valid().empty() ? std::uint8_t{1} : surface.valid()[index];
    write_field(cloud, logical_index, *valid, mask);
    dense = dense && mask == 1U;
    if (has_normals) {
      const auto normal = surface.normals()[index];
      write_field(cloud, logical_index, *normal_x, normal.x);
      write_field(cloud, logical_index, *normal_y, normal.y);
      write_field(cloud, logical_index, *normal_z, normal.z);
    }
    ++logical_index;
  });
  cloud.is_dense = dense;
  return cloud;
}

} // namespace

Result<OwnedSurface> read_ply_file(std::string_view path_utf8, LengthUnit unit,
                                   FrameId frame) noexcept {
  try {
    pcl::PCLPointCloud2 cloud;
    if (pcl::io::loadPLYFile(std::string(path_utf8), cloud) < 0) {
      return Result<OwnedSurface>::failure(
          io_error(ErrorCode::io_error, "failed to read PLY file", path_utf8));
    }
    return decode_cloud(cloud, unit, std::move(frame), path_utf8, "PLY");
  } catch (const std::exception& exception) {
    return Result<OwnedSurface>::failure(
        io_error(ErrorCode::io_error, "PLY backend read failed", path_utf8, exception.what()));
  } catch (...) {
    return Result<OwnedSurface>::failure(
        io_error(ErrorCode::io_error, "PLY backend read failed", path_utf8, "unknown exception"));
  }
}

Result<std::size_t> write_ply_file(std::string_view path_utf8, SurfaceView surface,
                                   bool binary) noexcept {
  try {
    const auto cloud = encode_cloud(surface);
    if (pcl::io::savePLYFile(std::string(path_utf8), cloud, Eigen::Vector4f::Zero(),
                             Eigen::Quaternionf::Identity(), binary, true) < 0) {
      return Result<std::size_t>::failure(
          io_error(ErrorCode::io_error, "failed to write PLY file", path_utf8));
    }
    return Result<std::size_t>::success(surface.size());
  } catch (const std::exception& exception) {
    return Result<std::size_t>::failure(
        io_error(ErrorCode::io_error, "PLY backend write failed", path_utf8, exception.what()));
  } catch (...) {
    return Result<std::size_t>::failure(
        io_error(ErrorCode::io_error, "PLY backend write failed", path_utf8, "unknown exception"));
  }
}

Result<OwnedSurface> read_pcd_file(std::string_view path_utf8, LengthUnit unit,
                                   FrameId frame) noexcept {
  try {
    pcl::PCLPointCloud2 cloud;
    if (pcl::io::loadPCDFile(std::string(path_utf8), cloud) < 0) {
      return Result<OwnedSurface>::failure(
          io_error(ErrorCode::io_error, "failed to read PCD file", path_utf8));
    }
    if (cloud.width == 0U || cloud.height == 0U || cloud.data.empty()) {
      return Result<OwnedSurface>::failure(
          io_error(ErrorCode::io_error, "PCD file contains no readable points", path_utf8));
    }
    return decode_cloud(cloud, unit, std::move(frame), path_utf8, "PCD");
  } catch (const std::exception& exception) {
    return Result<OwnedSurface>::failure(
        io_error(ErrorCode::io_error, "PCD backend read failed", path_utf8, exception.what()));
  } catch (...) {
    return Result<OwnedSurface>::failure(
        io_error(ErrorCode::io_error, "PCD backend read failed", path_utf8, "unknown exception"));
  }
}

Result<std::size_t> write_pcd_file(std::string_view path_utf8, SurfaceView surface,
                                   bool binary) noexcept {
  try {
    const auto cloud = encode_cloud(surface);
    if (pcl::io::savePCDFile(std::string(path_utf8), cloud, Eigen::Vector4f::Zero(),
                             Eigen::Quaternionf::Identity(), binary) < 0) {
      return Result<std::size_t>::failure(
          io_error(ErrorCode::io_error, "failed to write PCD file", path_utf8));
    }
    return Result<std::size_t>::success(surface.size());
  } catch (const std::exception& exception) {
    return Result<std::size_t>::failure(
        io_error(ErrorCode::io_error, "PCD backend write failed", path_utf8, exception.what()));
  } catch (...) {
    return Result<std::size_t>::failure(
        io_error(ErrorCode::io_error, "PCD backend write failed", path_utf8, "unknown exception"));
  }
}

} // namespace pointcloud_ad::backends::pcl_backend
