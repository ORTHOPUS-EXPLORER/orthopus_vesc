cmake_minimum_required(VERSION 3.25)
project(bench)

# Bench
add_executable(${PROJECT_NAME}
  bench.cpp
)

target_include_directories(${PROJECT_NAME}
PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include/
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
  lyra
  spdlog
  vescpp
)

set_target_properties(${PROJECT_NAME} 
PROPERTIES
  CXX_STANDARD 17
)
