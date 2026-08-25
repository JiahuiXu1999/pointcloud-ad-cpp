add_executable(
  pointcloud_ad_staged_benchmark
  EXCLUDE_FROM_ALL
  staged_benchmark.cpp
  ${PROJECT_SOURCE_DIR}/src/preprocess/validity.cpp
  ${PROJECT_SOURCE_DIR}/src/preprocess/voxel_sampling.cpp)
target_compile_features(pointcloud_ad_staged_benchmark PRIVATE cxx_std_20)
target_include_directories(
  pointcloud_ad_staged_benchmark
  PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src/preprocess)
target_link_libraries(pointcloud_ad_staged_benchmark PRIVATE nlohmann_json::nlohmann_json)
if(WIN32)
  target_link_libraries(pointcloud_ad_staged_benchmark PRIVATE Psapi)
endif()
pointcloudad_set_warnings(
  pointcloud_ad_staged_benchmark ${POINTCLOUDAD_ENABLE_WARNINGS_AS_ERRORS})
pointcloudad_enable_sanitizers(
  pointcloud_ad_staged_benchmark
  ${POINTCLOUDAD_ENABLE_ASAN}
  ${POINTCLOUDAD_ENABLE_UBSAN})
pointcloudad_enable_clang_tidy(
  pointcloud_ad_staged_benchmark ${POINTCLOUDAD_ENABLE_CLANG_TIDY})
