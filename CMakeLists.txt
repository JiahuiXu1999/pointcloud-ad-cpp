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

# Linux PCL/FLANN uses OpenMP even when the application is built with Clang.
# Require its compiler runtime up front instead of leaving unresolved GOMP/omp symbols.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  find_package(OpenMP REQUIRED COMPONENTS CXX)
endif()
find_package(PCL CONFIG REQUIRED COMPONENTS common features io kdtree search)
find_package(Threads REQUIRED)
find_package(lz4 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# vcpkg's static Linux PCL targets do not propagate FLANN's LZ4/OpenMP link closure. Keep the
# workaround private and reuse it for the library and tests that compile PCL backend sources.
add_library(pointcloudad_pcl_link_closure INTERFACE)
target_link_libraries(pointcloudad_pcl_link_closure INTERFACE lz4::lz4)
if(TARGET OpenMP::OpenMP_CXX)
  target_link_libraries(pointcloudad_pcl_link_closure INTERFACE OpenMP::OpenMP_CXX)
endif()

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
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # Private shared dependencies installed beside the SDK library remain relocatable.
  set_target_properties(pointcloud_ad PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()

target_sources(
  pointcloud_ad
  PRIVATE
    src/backends/pcl/pcl_backend.cpp
    src/backends/pcl/pcl_backend.hpp
    src/backends/pcl/pcl_comparison_backend.cpp
    src/backends/pcl/pcl_comparison_backend.hpp
    src/backends/pcl/pcl_cluster_backend.cpp
    src/backends/pcl/pcl_cluster_backend.hpp
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
    src/detection/defect_cluster.cpp
    src/detection/defect_cluster.hpp
    src/detection/defect_region.cpp
    src/detection/defect_region.hpp
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
    src/registration/backend_policy.hpp
    src/registration/cpu_executor.hpp
    src/registration/registration_engine.cpp
    src/registration/registration_gate.cpp
    src/registration/registration_gate.hpp
    src/pipeline/inspection_pipeline.cpp
    src/serialization/json_serialization.cpp
    src/serialization/json_serialization.hpp
    src/serialization/output_writer.cpp
    src/serialization/output_writer.hpp
  PUBLIC
    FILE_SET public_headers
    TYPE HEADERS
    BASE_DIRS include
    FILES
      include/pointcloud_ad/config.hpp
      include/pointcloud_ad/geometry.hpp
      include/pointcloud_ad/inspection_pipeline.hpp
      include/pointcloud_ad/inspection_context.hpp
      include/pointcloud_ad/inspection_result.hpp
      include/pointcloud_ad/normalization.hpp
      include/pointcloud_ad/registration_engine.hpp
      include/pointcloud_ad/registration_options.hpp
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
  pointcloud_ad PRIVATE POINTCLOUDAD_VERSION_STRING="${PROJECT_VERSION}")
target_include_directories(
  pointcloud_ad
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/backends/pcl
          ${CMAKE_CURRENT_SOURCE_DIR}/src/comparison
          ${CMAKE_CURRENT_SOURCE_DIR}/src/detection
          ${CMAKE_CURRENT_SOURCE_DIR}/src/io
          ${CMAKE_CURRENT_SOURCE_DIR}/src/pipeline
          ${CMAKE_CURRENT_SOURCE_DIR}/src/preprocess
          ${CMAKE_CURRENT_SOURCE_DIR}/src/registration
          ${CMAKE_CURRENT_SOURCE_DIR}/src/serialization
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(
  pointcloud_ad PRIVATE Threads::Threads pcl_common pcl_features pcl_io pcl_kdtree pcl_search
                         pointcloudad_pcl_link_closure nlohmann_json::nlohmann_json)

option(POINTCLOUDAD_ENABLE_CUDA "Enable resident CUDA registration" OFF)
if(POINTCLOUDAD_ENABLE_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit 13.2 REQUIRED)
  add_library(pointcloudad_cuda STATIC src/backends/cuda/cuda_correspondence.cu
    src/backends/cuda/cuda_correspondence.hpp src/backends/cuda/spatial_index.hpp)
  set_target_properties(pointcloudad_cuda PROPERTIES CUDA_STANDARD 20 CUDA_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON CUDA_RUNTIME_LIBRARY Static CUDA_VISIBILITY_PRESET hidden)
  target_include_directories(pointcloudad_cuda PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_options(pointcloudad_cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
  if(MSVC)
    target_compile_options(pointcloudad_cuda PRIVATE -Xcompiler=/W4,/permissive-,/Zc:__cplusplus,/utf-8)
    if(POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS)
      target_compile_options(pointcloudad_cuda PRIVATE -Xcompiler=/WX)
    endif()
  else()
    target_compile_options(pointcloudad_cuda PRIVATE -Xcompiler=-Wall,-Wextra,-Wconversion)
    if(POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS)
      target_compile_options(pointcloudad_cuda PRIVATE -Xcompiler=-Werror)
    endif()
  endif()
  target_link_libraries(pointcloudad_cuda PRIVATE CUDA::cudart_static)
  target_link_libraries(pointcloud_ad PRIVATE pointcloudad_cuda)
  target_include_directories(pointcloud_ad PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/backends/cuda)
  target_compile_definitions(pointcloud_ad PRIVATE POINTCLOUDAD_WITH_CUDA=1)
  find_file(POINTCLOUDAD_CUDA_EULA NAMES EULA.txt
    HINTS "${CUDAToolkit_LIBRARY_ROOT}" "${CUDAToolkit_BIN_DIR}/.." NO_DEFAULT_PATH REQUIRED)
  install(FILES "${POINTCLOUDAD_CUDA_EULA}" DESTINATION "share/licenses/nvidia-cuda")
endif()

pointcloudad_set_warnings(pointcloud_ad ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(pointcloud_ad ${POINTCLOUDAD_ENABLE_CLANG_TIDY})

# Baseline dispatch is compiled without ISA-specific options. AVX2 options are private
# and are attached only to the separate arithmetic implementation target.
option(POINTCLOUDAD_ENABLE_AVX2 "Build optional runtime-dispatched AVX2 arithmetic" ON)
add_library(pointcloudad_cpu STATIC src/registration/cpu_dispatch.cpp src/registration/cpu_dispatch.hpp
  src/registration/cpu_kernels.cpp src/registration/cpu_kernels.hpp src/registration/cpu_objective.hpp)
target_compile_features(pointcloudad_cpu PRIVATE cxx_std_20)
target_include_directories(pointcloudad_cpu PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(pointcloudad_cpu PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden)
set(pointcloudad_has_avx2 OFF)
if(POINTCLOUDAD_ENABLE_AVX2 AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(AMD64|amd64|x86_64|x86|i.86)$")
  if(MSVC OR CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(pointcloudad_has_avx2 ON)
    target_compile_definitions(pointcloudad_cpu PRIVATE POINTCLOUDAD_CAN_BUILD_AVX2=1)
    add_library(pointcloudad_avx2_options INTERFACE)
    if(MSVC)
      target_compile_options(pointcloudad_avx2_options INTERFACE /arch:AVX2 /fp:strict)
    else()
      target_compile_options(pointcloudad_avx2_options INTERFACE -mavx2 -ffp-contract=off)
    endif()
  endif()
endif()
if(MSVC)
  target_compile_options(pointcloudad_cpu PRIVATE /fp:strict)
else()
  target_compile_options(pointcloudad_cpu PRIVATE -ffp-contract=off)
endif()
if(pointcloudad_has_avx2)
  add_library(pointcloudad_avx2 OBJECT src/registration/cpu_kernels_avx2.cpp)
  target_compile_features(pointcloudad_avx2 PRIVATE cxx_std_20)
  target_include_directories(pointcloudad_avx2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(pointcloudad_avx2 PRIVATE pointcloudad_avx2_options)
  set_target_properties(pointcloudad_avx2 PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden)
  pointcloudad_set_warnings(pointcloudad_avx2 ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
  pointcloudad_enable_sanitizers(pointcloudad_avx2 ${POINTCLOUDAD_ENABLE_ASAN} ${POINTCLOUDAD_ENABLE_UBSAN})
  pointcloudad_enable_clang_tidy(pointcloudad_avx2 ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
  target_sources(pointcloudad_cpu PRIVATE $<TARGET_OBJECTS:pointcloudad_avx2>)
endif()
pointcloudad_set_warnings(pointcloudad_cpu ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(pointcloudad_cpu ${POINTCLOUDAD_ENABLE_ASAN} ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(pointcloudad_cpu ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
target_link_libraries(pointcloud_ad PRIVATE pointcloudad_cpu)

if(POINTCLOUDAD_BUILD_CLI)
  add_executable(
    pcad
    src/cli/main.cpp
    src/io/ply_adapter.cpp
    src/io/pcd_adapter.cpp
    src/backends/pcl/pcl_io_backend.cpp
    src/serialization/json_serialization.cpp
    src/serialization/output_writer.cpp)
  target_compile_features(pcad PRIVATE cxx_std_20)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    file(RELATIVE_PATH pointcloudad_cli_library_path
      "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
      "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
    set_target_properties(pcad PROPERTIES
      INSTALL_RPATH "$ORIGIN/${pointcloudad_cli_library_path}")
  endif()
  target_include_directories(
    pcad
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/io
            ${CMAKE_CURRENT_SOURCE_DIR}/src/backends/pcl
            ${CMAKE_CURRENT_SOURCE_DIR}/src/serialization)
  target_link_libraries(
    pcad
    PRIVATE PointCloudAD::pointcloud_ad pcl_common pcl_io pointcloudad_pcl_link_closure
            nlohmann_json::nlohmann_json)
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

if(PROJECT_IS_TOP_LEVEL)
  add_subdirectory(benchmarks EXCLUDE_FROM_ALL)
endif()

set(pointcloudad_install_cmakedir
    "${CMAKE_INSTALL_LIBDIR}/cmake/PointCloudAD")

install(
  TARGETS pointcloud_ad
  EXPORT PointCloudADTargets
  RUNTIME_DEPENDENCY_SET pointcloudad_runtime_dependencies
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  FILE_SET public_headers DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/pointcloud_ad/export.hpp"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/pointcloud_ad")

if(POINTCLOUDAD_BUILD_CLI)
  install(
    TARGETS pcad
    RUNTIME_DEPENDENCY_SET pointcloudad_runtime_dependencies
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()

install(
  RUNTIME_DEPENDENCY_SET pointcloudad_runtime_dependencies
  PRE_EXCLUDE_REGEXES "api-ms-.*" "ext-ms-.*"
  POST_EXCLUDE_REGEXES ".*[Ss]ystem32.*" "^/lib/.*" "^/usr/lib/.*"
  DIRECTORIES "$<TARGET_FILE_DIR:pcl_common>" "$<TARGET_FILE_DIR:pcl_io>"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}")

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

if(MSVC)
  set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION "${CMAKE_INSTALL_BINDIR}")
  set(CMAKE_INSTALL_UCRT_LIBRARIES TRUE)
  include(InstallRequiredSystemLibraries)
endif()

install(
  FILES LICENSE README.md SECURITY.md THIRD_PARTY_NOTICES.md
  DESTINATION ".")
install(FILES docs/SDK.md DESTINATION "docs")
install(FILES docs/architecture/HETEROGENEOUS_REGISTRATION.md docs/architecture/NUMERICAL_REPRODUCIBILITY.md DESTINATION "docs/architecture")
install(FILES docs/PERFORMANCE_PLAN.md DESTINATION "docs")
install(FILES docs/adr/0010-heterogeneous-registration.md DESTINATION "docs/adr")
install(
  FILES docs/benchmarks/PCAD-HET-001.md docs/benchmarks/PCAD-HET-001.csv
    docs/benchmarks/PCAD-HET-002.md docs/benchmarks/PCAD-HET-002-search.csv
    docs/benchmarks/PCAD-HET-002-registration.csv docs/benchmarks/PCAD-HET-003.md
    docs/benchmarks/PCAD-HET-003.csv docs/benchmarks/PCAD-SIMD-003.md
    docs/benchmarks/PCAD-PERF-001.md docs/benchmarks/PCAD-PERF-001-baseline.csv
    docs/benchmarks/PCAD-PERF-001-100k.csv docs/benchmarks/PCAD-PERF-002-scalar.csv
    docs/benchmarks/PCAD-SIMD-003-65-1-scalar.csv docs/benchmarks/PCAD-SIMD-003-65-1-auto.csv
    docs/benchmarks/PCAD-SIMD-003-65-4-scalar.csv docs/benchmarks/PCAD-SIMD-003-65-4-auto.csv
    docs/benchmarks/PCAD-SIMD-003-317-1-scalar.csv docs/benchmarks/PCAD-SIMD-003-317-1-auto.csv
    docs/benchmarks/PCAD-SIMD-003-317-4-scalar.csv docs/benchmarks/PCAD-SIMD-003-317-4-auto.csv
    docs/benchmarks/PCAD-SIMD-003-1m.csv docs/benchmarks/PCAD-SIMD-003-kernels.csv
  DESTINATION "docs/benchmarks")
install(
  FILES
    docs/benchmarks/PCAD-GPU-003-1000-transfers.txt
    docs/benchmarks/PCAD-GPU-003-1000.csv
    docs/benchmarks/PCAD-GPU-003-317-transfers.txt
    docs/benchmarks/PCAD-GPU-003-317.csv
    docs/benchmarks/PCAD-GPU-003-65-transfers.txt
    docs/benchmarks/PCAD-GPU-003-65.csv
    docs/benchmarks/PCAD-GPU-003.md
    docs/benchmarks/PCAD-PIPEPERF-001.csv
    docs/benchmarks/PCAD-PIPEPERF-002.csv
    docs/benchmarks/PCAD-PIPEPERF-003-1000-1.csv
    docs/benchmarks/PCAD-PIPEPERF-003-1000-4.csv
    docs/benchmarks/PCAD-PIPEPERF-003-317-1.csv
    docs/benchmarks/PCAD-PIPEPERF-003-317-4.csv
    docs/benchmarks/PCAD-PIPEPERF-003-65-1.csv
    docs/benchmarks/PCAD-PIPEPERF-003-65-4.csv
    docs/benchmarks/PCAD-PIPEPERF-003-defect.csv
    docs/benchmarks/PCAD-PIPEPERF-003.md
  DESTINATION "docs/benchmarks")

install(
  FILES
    docs/validation/PCAD-VALID-001-ground-truth.json
    docs/validation/PCAD-VALID-001-linux-clang.json
    docs/validation/PCAD-VALID-001-linux-gcc.json
    docs/validation/PCAD-VALID-001-windows-cpu.json
    docs/validation/PCAD-VALID-001-windows-cuda-large.json
    docs/validation/PCAD-VALID-001-windows-cuda.json
    docs/validation/PCAD-VALID-001-windows-scalar.json
    docs/validation/PCAD-VALID-001.md
  DESTINATION "docs/validation")

install(FILES docs/ROADMAP.md DESTINATION "docs")
install(FILES
    docs/release/M16.md
    docs/release/M16-windows-cpu-package.json
    docs/release/M16-windows-cuda-package.json
    docs/release/M16-linux-package.json
    DESTINATION "docs/release")
install(PROGRAMS scripts/validate-generated.py DESTINATION "scripts")

install(
  DIRECTORY examples/sdk_consumer
  DESTINATION "examples"
  FILES_MATCHING
  PATTERN "CMakeLists.txt"
  PATTERN "*.cpp")

if(DEFINED VCPKG_INSTALLED_DIR AND DEFINED VCPKG_TARGET_TRIPLET)
  set(pointcloudad_vcpkg_share
      "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/share")
  if(EXISTS "${pointcloudad_vcpkg_share}")
    install(
      DIRECTORY "${pointcloudad_vcpkg_share}/"
      DESTINATION "share/licenses"
      FILES_MATCHING
      PATTERN "copyright")
  endif()
endif()

if(PROJECT_IS_TOP_LEVEL)
  include(FormatTargets)
  pointcloudad_add_format_targets()

  if(WIN32)
    set(pointcloudad_package_platform "windows-x86_64")
    set(CPACK_GENERATOR "ZIP")
  elseif(APPLE)
    set(pointcloudad_package_platform "macos-${CMAKE_SYSTEM_PROCESSOR}")
    set(CPACK_GENERATOR "TGZ")
  else()
    set(pointcloudad_package_platform "linux-${CMAKE_SYSTEM_PROCESSOR}")
    set(CPACK_GENERATOR "TGZ")
  endif()
  set(CPACK_PACKAGE_NAME "PointCloudAD-SDK")
  set(CPACK_PACKAGE_VENDOR "JiahuiXu1999")
  set(CPACK_PACKAGE_CONTACT "JiahuiXu1999")
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "PointCloudAD deterministic industrial inspection SDK")
  set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
  set(CPACK_PACKAGE_FILE_NAME
      "pointcloud-ad-sdk-${PROJECT_VERSION}-${pointcloudad_package_platform}")
  set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
  set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
  include(CPack)
endif()
