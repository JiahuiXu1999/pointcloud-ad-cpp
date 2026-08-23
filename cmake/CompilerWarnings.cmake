function(pointcloudad_set_warnings target warnings_as_errors)
  if(MSVC)
    set(warnings /W4 /permissive- /Zc:__cplusplus /utf-8)
    if(warnings_as_errors)
      list(APPEND warnings /WX)
    endif()
  else()
    set(
      warnings
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wsign-conversion
      -Wshadow
      -Wnon-virtual-dtor
      -Wold-style-cast
      -Woverloaded-virtual)
    if(warnings_as_errors)
      list(APPEND warnings -Werror)
    endif()
  endif()

  target_compile_options(${target} PRIVATE ${warnings})
endfunction()

