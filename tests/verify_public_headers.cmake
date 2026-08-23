file(
  GLOB pointcloudad_public_headers
  LIST_DIRECTORIES FALSE
  "${POINTCLOUDAD_SOURCE_DIR}/include/pointcloud_ad/*.hpp")

if(NOT pointcloudad_public_headers)
  message(FATAL_ERROR "No public PointCloudAD headers were found.")
endif()

foreach(header IN LISTS pointcloudad_public_headers)
  file(READ "${header}" contents)
  if(contents MATCHES "#[ \t]*include[ \t]*[<\"]((pcl|Eigen|vtk|boost)/|filesystem|.*cli)")
    message(FATAL_ERROR "Forbidden public dependency in ${header}")
  endif()
endforeach()
