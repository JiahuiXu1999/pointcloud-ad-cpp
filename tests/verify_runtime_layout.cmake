foreach(required_path IN ITEMS POINTCLOUDAD_LIBRARY POINTCLOUDAD_LINKER_FILE
                               POINTCLOUDAD_UNIT_TEST POINTCLOUDAD_CLI)
  if(NOT DEFINED ${required_path} OR NOT EXISTS "${${required_path}}")
    message(FATAL_ERROR "Missing build artifact ${required_path}: '${${required_path}}'")
  endif()
endforeach()

if(CMAKE_HOST_WIN32)
  cmake_path(GET POINTCLOUDAD_LIBRARY PARENT_PATH library_directory)
  cmake_path(GET POINTCLOUDAD_UNIT_TEST PARENT_PATH unit_test_directory)
  cmake_path(GET POINTCLOUDAD_CLI PARENT_PATH cli_directory)

  if(NOT library_directory STREQUAL unit_test_directory OR
     NOT library_directory STREQUAL cli_directory)
    message(
      FATAL_ERROR
        "The DLL, tests, and CLI must share one runtime directory. "
        "DLL='${library_directory}', tests='${unit_test_directory}', CLI='${cli_directory}'")
  endif()

  if(NOT POINTCLOUDAD_LIBRARY MATCHES "\\.dll$" OR
     NOT POINTCLOUDAD_LINKER_FILE MATCHES "\\.lib$")
    message(
      FATAL_ERROR
        "Windows must produce a DLL and import library: "
        "'${POINTCLOUDAD_LIBRARY}', '${POINTCLOUDAD_LINKER_FILE}'")
  endif()
endif()
