cmake_minimum_required(VERSION 3.25)
project(PointCloudADConsumer LANGUAGES CXX)

find_package(PointCloudAD 0.1 CONFIG REQUIRED)

add_executable(pointcloud_ad_consumer smoke.cpp)
target_compile_features(pointcloud_ad_consumer PRIVATE cxx_std_20)
target_link_libraries(pointcloud_ad_consumer PRIVATE PointCloudAD::pointcloud_ad)

if(WIN32)
  add_custom_command(
    TARGET pointcloud_ad_consumer
    POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            $<TARGET_RUNTIME_DLLS:pointcloud_ad_consumer>
            $<TARGET_FILE_DIR:pointcloud_ad_consumer>
    COMMAND_EXPAND_LISTS
    COMMENT "Deploying installed PointCloudAD runtime dependencies")
endif()
