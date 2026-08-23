function(pointcloudad_enable_clang_tidy target enabled)
  if(NOT enabled)
    return()
  endif()

  find_program(POINTCLOUDAD_CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
  set_target_properties(
    ${target}
    PROPERTIES CXX_CLANG_TIDY
               "${POINTCLOUDAD_CLANG_TIDY_EXECUTABLE};--warnings-as-errors=*")
endfunction()

