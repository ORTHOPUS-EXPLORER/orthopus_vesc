project(orthopus_vesc)

find_package(spdlog 1.9.2 REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/common.cpp
  src/device.cpp
  src/host.cpp
  src/target.cpp
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

install(TARGETS 
  ${PROJECT_NAME}
)
install(DIRECTORY 
  ${CMAKE_CURRENT_LIST_DIR}/include/
  DESTINATION include
)

if(DEFINED ENV{AMENT_PREFIX_PATH})
  find_package(ament_cmake REQUIRED)


  install(
    TARGETS 
      ${PROJECT_NAME} 
      vescpp 
      heatshrink
  )
  
  install(
    DIRECTORY   include/
    DESTINATION include
  )

  ament_export_dependencies(nlohmann_json spdlog)

  ament_export_include_directories(
    include/
  )

  ament_export_libraries(
    ${PROJECT_NAME} 
    vescpp
    heatshrink
  )


  ament_package()
endif()