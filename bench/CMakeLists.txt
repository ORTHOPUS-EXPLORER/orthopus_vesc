cmake_minimum_required(VERSION 3.22)
project(bench)

# Bench
add_executable(${PROJECT_NAME}
  bench.cpp
)

target_include_directories(${PROJECT_NAME}
PRIVATE
  include/
)

target_link_libraries(${PROJECT_NAME}
PRIVATE
  veschost
  lyra
)

set_target_properties(${PROJECT_NAME} 
PROPERTIES
  CXX_STANDARD 17
)
