project(veschost)

find_package(spdlog 1.9.2 REQUIRED)

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

install(TARGETS 
  ${PROJECT_NAME}
)
install(DIRECTORY 
  ${CMAKE_CURRENT_LIST_DIR}/include/
  DESTINATION include
)

if(DEFINED ENV{AMENT_PREFIX_PATH})
  find_package(ament_cmake REQUIRED)

  set_target_properties(spdlog PROPERTIES POSITION_INDEPENDENT_CODE ON)

  install(
    TARGETS 
      ${PROJECT_NAME} 
      vescpp 
      spdlog
      heatshrink
      nlohmann_json
    EXPORT export_${PROJECT_NAME} 
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

  ament_export_targets(
    export_${PROJECT_NAME}
  )

  ament_package()
endif()