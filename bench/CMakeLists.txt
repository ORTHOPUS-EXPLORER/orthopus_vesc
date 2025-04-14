cmake_minimum_required(VERSION 3.22)
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
  veschost
  lyra
)

set_target_properties(${PROJECT_NAME} 
PROPERTIES
  CXX_STANDARD 17
)
