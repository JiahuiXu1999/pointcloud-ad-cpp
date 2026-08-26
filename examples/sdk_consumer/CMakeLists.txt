cmake_minimum_required(VERSION 3.25)
project(PointCloudADSdkConsumer LANGUAGES CXX)

find_package(PointCloudAD 0.1 CONFIG REQUIRED)

add_executable(pointcloud_ad_sdk_consumer main.cpp)
target_compile_features(pointcloud_ad_sdk_consumer PRIVATE cxx_std_20)
target_link_libraries(pointcloud_ad_sdk_consumer PRIVATE PointCloudAD::pointcloud_ad)
