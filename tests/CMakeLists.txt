add_executable(pointcloud_ad_unit_tests unit/version_test.cpp)
target_compile_features(pointcloud_ad_unit_tests PRIVATE cxx_std_20)
target_link_libraries(pointcloud_ad_unit_tests PRIVATE PointCloudAD::pointcloud_ad)

pointcloudad_set_warnings(
  pointcloud_ad_unit_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_unit_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_unit_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})

add_test(NAME pointcloud_ad.unit.version COMMAND pointcloud_ad_unit_tests)

add_executable(pointcloud_ad_result_tests unit/result_test.cpp)
target_compile_features(pointcloud_ad_result_tests PRIVATE cxx_std_20)
target_link_libraries(pointcloud_ad_result_tests PRIVATE PointCloudAD::pointcloud_ad)
pointcloudad_set_warnings(
  pointcloud_ad_result_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_result_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_result_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.result COMMAND pointcloud_ad_result_tests)

add_executable(pointcloud_ad_geometry_tests unit/geometry_test.cpp)
target_compile_features(pointcloud_ad_geometry_tests PRIVATE cxx_std_20)
target_link_libraries(pointcloud_ad_geometry_tests PRIVATE PointCloudAD::pointcloud_ad)
pointcloudad_set_warnings(
  pointcloud_ad_geometry_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_geometry_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_geometry_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.geometry COMMAND pointcloud_ad_geometry_tests)

add_executable(pointcloud_ad_config_tests unit/config_test.cpp)
target_compile_features(pointcloud_ad_config_tests PRIVATE cxx_std_20)
target_link_libraries(pointcloud_ad_config_tests PRIVATE PointCloudAD::pointcloud_ad)
pointcloudad_set_warnings(
  pointcloud_ad_config_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_config_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_config_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.config COMMAND pointcloud_ad_config_tests)

add_executable(pointcloud_ad_surface_tests unit/surface_test.cpp)
target_compile_features(pointcloud_ad_surface_tests PRIVATE cxx_std_20)
target_link_libraries(pointcloud_ad_surface_tests PRIVATE PointCloudAD::pointcloud_ad)
pointcloudad_set_warnings(
  pointcloud_ad_surface_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_surface_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_surface_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.surface COMMAND pointcloud_ad_surface_tests)

add_executable(pointcloud_ad_normalization_tests unit/normalization_test.cpp)
target_compile_features(pointcloud_ad_normalization_tests PRIVATE cxx_std_20)
target_link_libraries(pointcloud_ad_normalization_tests PRIVATE PointCloudAD::pointcloud_ad)
pointcloudad_set_warnings(
  pointcloud_ad_normalization_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_normalization_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_normalization_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.normalization COMMAND pointcloud_ad_normalization_tests)

add_executable(pointcloud_ad_registration_tests unit/registration_test.cpp)
target_compile_features(pointcloud_ad_registration_tests PRIVATE cxx_std_20)
target_link_libraries(pointcloud_ad_registration_tests PRIVATE PointCloudAD::pointcloud_ad)
pointcloudad_set_warnings(
  pointcloud_ad_registration_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_registration_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_registration_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.registration COMMAND pointcloud_ad_registration_tests)

add_executable(
  pointcloud_ad_registration_icp_tests
  unit/registration_icp_test.cpp
  ${PROJECT_SOURCE_DIR}/src/backends/pcl/pcl_registration_backend.cpp
  ${PROJECT_SOURCE_DIR}/src/registration/registration_gate.cpp)
target_compile_features(pointcloud_ad_registration_icp_tests PRIVATE cxx_std_20)
target_include_directories(
  pointcloud_ad_registration_icp_tests
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src/backends/pcl
    ${PROJECT_SOURCE_DIR}/src/registration)
target_link_libraries(
  pointcloud_ad_registration_icp_tests
  PRIVATE pcl_common pcl_kdtree pcl_search)
pointcloudad_set_warnings(
  pointcloud_ad_registration_icp_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_registration_icp_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_registration_icp_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.registration_icp COMMAND pointcloud_ad_registration_icp_tests)

add_executable(
  pointcloud_ad_registration_gate_tests
  unit/registration_gate_test.cpp
  ${PROJECT_SOURCE_DIR}/src/registration/registration_gate.cpp)
target_compile_features(pointcloud_ad_registration_gate_tests PRIVATE cxx_std_20)
target_include_directories(
  pointcloud_ad_registration_gate_tests
  PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src/registration)
target_link_libraries(pointcloud_ad_registration_gate_tests PRIVATE PointCloudAD::pointcloud_ad)
pointcloudad_set_warnings(
  pointcloud_ad_registration_gate_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_registration_gate_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_registration_gate_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.registration_gate COMMAND pointcloud_ad_registration_gate_tests)

add_executable(
  pointcloud_ad_pcl_backend_tests
  unit/pcl_backend_test.cpp
  ${PROJECT_SOURCE_DIR}/src/backends/pcl/pcl_backend.cpp)
target_compile_features(pointcloud_ad_pcl_backend_tests PRIVATE cxx_std_20)
target_include_directories(
  pointcloud_ad_pcl_backend_tests
  PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src/backends/pcl)
target_link_libraries(pointcloud_ad_pcl_backend_tests PRIVATE pcl_common)
pointcloudad_set_warnings(
  pointcloud_ad_pcl_backend_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_pcl_backend_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_pcl_backend_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.pcl_backend COMMAND pointcloud_ad_pcl_backend_tests)

add_executable(
  pointcloud_ad_ply_adapter_tests
  unit/ply_adapter_test.cpp
  ${PROJECT_SOURCE_DIR}/src/io/ply_adapter.cpp
  ${PROJECT_SOURCE_DIR}/src/backends/pcl/pcl_io_backend.cpp)
target_compile_features(pointcloud_ad_ply_adapter_tests PRIVATE cxx_std_20)
target_include_directories(
  pointcloud_ad_ply_adapter_tests
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src/io
    ${PROJECT_SOURCE_DIR}/src/backends/pcl)
target_link_libraries(pointcloud_ad_ply_adapter_tests PRIVATE pcl_common pcl_io)
pointcloudad_set_warnings(
  pointcloud_ad_ply_adapter_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_ply_adapter_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_ply_adapter_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.ply_adapter COMMAND pointcloud_ad_ply_adapter_tests)

add_executable(
  pointcloud_ad_pcd_adapter_tests
  unit/pcd_adapter_test.cpp
  ${PROJECT_SOURCE_DIR}/src/io/pcd_adapter.cpp
  ${PROJECT_SOURCE_DIR}/src/backends/pcl/pcl_io_backend.cpp)
target_compile_features(pointcloud_ad_pcd_adapter_tests PRIVATE cxx_std_20)
target_include_directories(
  pointcloud_ad_pcd_adapter_tests
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src/io
    ${PROJECT_SOURCE_DIR}/src/backends/pcl)
target_link_libraries(pointcloud_ad_pcd_adapter_tests PRIVATE pcl_common pcl_io)
pointcloudad_set_warnings(
  pointcloud_ad_pcd_adapter_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_pcd_adapter_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_pcd_adapter_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.pcd_adapter COMMAND pointcloud_ad_pcd_adapter_tests)

add_executable(
  pointcloud_ad_validity_tests
  unit/validity_test.cpp
  ${PROJECT_SOURCE_DIR}/src/preprocess/validity.cpp)
target_compile_features(pointcloud_ad_validity_tests PRIVATE cxx_std_20)
target_include_directories(
  pointcloud_ad_validity_tests
  PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src/preprocess)
pointcloudad_set_warnings(
  pointcloud_ad_validity_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_validity_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_validity_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.validity COMMAND pointcloud_ad_validity_tests)

add_executable(
  pointcloud_ad_voxel_sampling_tests
  unit/voxel_sampling_test.cpp
  ${PROJECT_SOURCE_DIR}/src/preprocess/voxel_sampling.cpp)
target_compile_features(pointcloud_ad_voxel_sampling_tests PRIVATE cxx_std_20)
target_include_directories(
  pointcloud_ad_voxel_sampling_tests
  PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src/preprocess)
pointcloudad_set_warnings(
  pointcloud_ad_voxel_sampling_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_voxel_sampling_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_voxel_sampling_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.voxel_sampling COMMAND pointcloud_ad_voxel_sampling_tests)

add_executable(
  pointcloud_ad_normal_boundary_tests
  unit/normal_boundary_test.cpp
  ${PROJECT_SOURCE_DIR}/src/preprocess/normal_boundary.cpp
  ${PROJECT_SOURCE_DIR}/src/backends/pcl/pcl_preprocess_backend.cpp)
target_compile_features(pointcloud_ad_normal_boundary_tests PRIVATE cxx_std_20)
target_include_directories(
  pointcloud_ad_normal_boundary_tests
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src/preprocess
    ${PROJECT_SOURCE_DIR}/src/backends/pcl)
target_link_libraries(
  pointcloud_ad_normal_boundary_tests
  PRIVATE pcl_common pcl_features pcl_kdtree pcl_search)
pointcloudad_set_warnings(
  pointcloud_ad_normal_boundary_tests ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_normal_boundary_tests
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_normal_boundary_tests ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
add_test(NAME pointcloud_ad.unit.normal_boundary COMMAND pointcloud_ad_normal_boundary_tests)

add_test(
  NAME pointcloud_ad.contract.public_headers
  COMMAND
    "${CMAKE_COMMAND}"
    "-DPOINTCLOUDAD_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/verify_public_headers.cmake")

if(TARGET pcad)
  add_test(
    NAME pointcloud_ad.build.runtime_layout
    COMMAND
      "${CMAKE_COMMAND}"
      "-DPOINTCLOUDAD_LIBRARY=$<TARGET_FILE:pointcloud_ad>"
      "-DPOINTCLOUDAD_LINKER_FILE=$<TARGET_LINKER_FILE:pointcloud_ad>"
      "-DPOINTCLOUDAD_UNIT_TEST=$<TARGET_FILE:pointcloud_ad_unit_tests>"
      "-DPOINTCLOUDAD_CLI=$<TARGET_FILE:pcad>"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/verify_runtime_layout.cmake")

  add_test(NAME pointcloud_ad.cli.version COMMAND pcad --version)
  set_tests_properties(
    pointcloud_ad.cli.version PROPERTIES PASS_REGULAR_EXPRESSION
                                          "pcad 0\\.1\\.0-dev")
endif()
