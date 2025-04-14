add_library(veschost STATIC
  veschost.cpp
)

target_include_directories(veschost
PUBLIC
  include/
)

target_link_libraries(veschost
PUBLIC
  vescpp
  spdlog
)