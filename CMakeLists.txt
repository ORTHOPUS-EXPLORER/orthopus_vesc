cmake_minimum_required(VERSION 3.22)
project(orthopus_vesc)

set(VESCPP_BUILD_EXAMPLE OFF)
set(VESCPP_BUILD_TESTS   OFF)
add_subdirectory(lib/vescpp/)

add_subdirectory(orthopus_vesc/)

if(NOT DEFINED ENV{AMENT_PREFIX_PATH})
  # Lyra is used in all examples so fetch it here
  if(NOT TARGET lyra)
    FetchContent_Declare(lyra
      GIT_REPOSITORY  https://github.com/bfgroup/Lyra
      GIT_TAG         1.6.1
      GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
      FIND_PACKAGE_ARGS 1.6.1
    )
    FetchContent_MakeAvailable(lyra)
    if(NOT lyra_FOUND)
      message(STATUS "[+] Use FetchContent to get lyra 1.6.1")
    else()
      message(STATUS "[+] Use System-provided lyra ${lyra_VERSION}")
    endif()
  endif()
  add_subdirectory(bench/)
  add_subdirectory(tests/)
endif()
