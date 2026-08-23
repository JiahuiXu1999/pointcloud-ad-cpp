function(pointcloudad_add_format_targets)
  find_program(POINTCLOUDAD_CLANG_FORMAT_EXECUTABLE NAMES clang-format)
  if(NOT POINTCLOUDAD_CLANG_FORMAT_EXECUTABLE)
    message(STATUS "clang-format not found; format targets are unavailable")
    return()
  endif()

  file(
    GLOB_RECURSE pointcloudad_format_files
    CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/include/*.hpp"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.hpp"
    "${PROJECT_SOURCE_DIR}/tests/*.cpp"
    "${PROJECT_SOURCE_DIR}/tests/*.hpp")

  add_custom_target(
    format
    COMMAND "${POINTCLOUDAD_CLANG_FORMAT_EXECUTABLE}" -i
            ${pointcloudad_format_files}
    COMMENT "Formatting PointCloudAD sources")
  add_custom_target(
    format-check
    COMMAND "${POINTCLOUDAD_CLANG_FORMAT_EXECUTABLE}" --dry-run --Werror
            ${pointcloudad_format_files}
    COMMENT "Checking PointCloudAD source formatting")
endfunction()

