cmake_minimum_required(VERSION 3.25)

project(
  PointCloudAD
  VERSION 0.1.0
  DESCRIPTION "Deterministic point-cloud anomaly detection for industrial inspection"
  LANGUAGES CXX)

include(CTest)
include(CMakePackageConfigHelpers)
include(GenerateExportHeader)
include(GNUInstallDirs)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerWarnings)
include(Sanitizers)
include(StaticAnalysis)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

find_package(PCL CONFIG REQUIRED COMPONENTS common features io kdtree search)

option(POINTCLOUDAD_BUILD_CLI "Build the pcad command-line application" ON)
option(POINTCLOUDAD_BUILD_TESTS "Build PointCloudAD tests" ${PROJECT_IS_TOP_LEVEL})
option(POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(POINTCLOUDAD_ENABLE_CLANG_TIDY "Run clang-tidy during compilation" OFF)
option(POINTCLOUDAD_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(POINTCLOUDAD_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)

if(PROJECT_IS_TOP_LEVEL)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# Keep every Windows runtime artifact together so executables can load the DLL
# from both the build tree and an installed package without PATH manipulation.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")

add_library(pointcloud_ad SHARED)
add_library(PointCloudAD::pointcloud_ad ALIAS pointcloud_ad)

target_sources(
  pointcloud_ad
  PRIVATE
    src/backends/pcl/pcl_backend.cpp
    src/backends/pcl/pcl_backend.hpp
    src/backends/pcl/pcl_comparison_backend.cpp
    src/backends/pcl/pcl_comparison_backend.hpp
    src/backends/pcl/pcl_io_backend.cpp
    src/backends/pcl/pcl_io_backend.hpp
    src/backends/pcl/pcl_preprocess_backend.cpp
    src/backends/pcl/pcl_preprocess_backend.hpp
    src/backends/pcl/pcl_registration_backend.cpp
    src/backends/pcl/pcl_registration_backend.hpp
    src/comparison/comparison_statistics.cpp
    src/comparison/comparison_statistics.hpp
    src/comparison/coverage_field.cpp
    src/comparison/coverage_field.hpp
    src/comparison/deviation_field.cpp
    src/comparison/deviation_field.hpp
    src/core/version.cpp
    src/io/pcd_adapter.cpp
    src/io/pcd_adapter.hpp
    src/io/ply_adapter.cpp
    src/io/ply_adapter.hpp
    src/preprocess/validity.cpp
    src/preprocess/validity.hpp
    src/preprocess/normal_boundary.cpp
    src/preprocess/normal_boundary.hpp
    src/preprocess/voxel_sampling.cpp
    src/preprocess/voxel_sampling.hpp
    src/registration/registration_gate.cpp
    src/registration/registration_gate.hpp
  PUBLIC
    FILE_SET public_headers
    TYPE HEADERS
    BASE_DIRS include
    FILES
      include/pointcloud_ad/config.hpp
      include/pointcloud_ad/geometry.hpp
      include/pointcloud_ad/normalization.hpp
      include/pointcloud_ad/registration.hpp
      include/pointcloud_ad/result.hpp
      include/pointcloud_ad/status.hpp
      include/pointcloud_ad/surface.hpp
      include/pointcloud_ad/version.hpp)

generate_export_header(
  pointcloud_ad
  EXPORT_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/generated/pointcloud_ad/export.hpp")

target_compile_features(pointcloud_ad PUBLIC cxx_std_20)
set_target_properties(
  pointcloud_ad
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN YES
             WINDOWS_EXPORT_ALL_SYMBOLS OFF)
target_compile_definitions(
  pointcloud_ad PRIVATE POINTCLOUDAD_VERSION_STRING="${PROJECT_VERSION}-dev")
target_include_directories(
  pointcloud_ad
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/backends/pcl
          ${CMAKE_CURRENT_SOURCE_DIR}/src/comparison
          ${CMAKE_CURRENT_SOURCE_DIR}/src/io
          ${CMAKE_CURRENT_SOURCE_DIR}/src/preprocess
          ${CMAKE_CURRENT_SOURCE_DIR}/src/registration
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(
  pointcloud_ad PRIVATE pcl_common pcl_features pcl_io pcl_kdtree pcl_search)

pointcloudad_set_warnings(pointcloud_ad ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(pointcloud_ad ${POINTCLOUDAD_ENABLE_CLANG_TIDY})

if(POINTCLOUDAD_BUILD_CLI)
  add_executable(pcad src/cli/main.cpp)
  target_compile_features(pcad PRIVATE cxx_std_20)
  target_link_libraries(pcad PRIVATE PointCloudAD::pointcloud_ad)
  pointcloudad_set_warnings(pcad ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
  pointcloudad_enable_sanitizers(
    pcad
    ${POINTCLOUDAD_ENABLE_ASAN}
    ${POINTCLOUDAD_ENABLE_UBSAN})
  pointcloudad_enable_clang_tidy(pcad ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
endif()

if(BUILD_TESTING AND POINTCLOUDAD_BUILD_TESTS)
  add_subdirectory(tests)
endif()

set(pointcloudad_install_cmakedir
    "${CMAKE_INSTALL_LIBDIR}/cmake/PointCloudAD")

install(
  TARGETS pointcloud_ad
  EXPORT PointCloudADTargets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  FILE_SET public_headers DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/pointcloud_ad/export.hpp"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/pointcloud_ad")

if(POINTCLOUDAD_BUILD_CLI)
  install(TARGETS pcad RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()

configure_package_config_file(
  cmake/PointCloudADConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/PointCloudADConfig.cmake"
  INSTALL_DESTINATION "${pointcloudad_install_cmakedir}")
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/PointCloudADConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion)

install(
  EXPORT PointCloudADTargets
  FILE PointCloudADTargets.cmake
  NAMESPACE PointCloudAD::
  DESTINATION "${pointcloudad_install_cmakedir}")
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/PointCloudADConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/PointCloudADConfigVersion.cmake"
  DESTINATION "${pointcloudad_install_cmakedir}")

if(PROJECT_IS_TOP_LEVEL)
  include(FormatTargets)
  pointcloudad_add_format_targets()
endif()
