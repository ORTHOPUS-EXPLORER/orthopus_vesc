project(veschost)

find_package(spdlog REQUIRED)

add_library(${PROJECT_NAME} STATIC
  veschost.cpp
)

target_include_directories(${PROJECT_NAME}
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
  vescpp
  spdlog::spdlog
)

set_target_properties(${PROJECT_NAME}
PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)