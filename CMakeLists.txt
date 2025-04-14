cmake_minimum_required(VERSION 3.22)
project(VESCHost)

include(FetchContent)
FetchContent_Declare(spdlog
  GIT_REPOSITORY  https://github.com/gabime/spdlog
  GIT_TAG         v1.10.0
)
FetchContent_Declare(lyra
  GIT_REPOSITORY  https://github.com/bfgroup/Lyra
  GIT_TAG         1.6.1
)

FetchContent_MakeAvailable(
  spdlog
  lyra
)

set(VESCPP_BUILD_EXAMPLE OFF)
set(VESCPP_BUILD_TESTS   OFF)
add_subdirectory(lib/vescpp/)

add_subdirectory(veschost/)

if($ENV{COLCON_PREFIX_PATH})
  add_subdirectory(ros/)
else()
  add_subdirectory(bench/)
  add_subdirectory(tests/)
endif()
