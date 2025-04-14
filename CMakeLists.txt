cmake_minimum_required(VERSION 3.22)
project(veschost)

set(VESCPP_BUILD_EXAMPLE OFF)
set(VESCPP_BUILD_TESTS   OFF)
add_subdirectory(lib/vescpp/)

add_subdirectory(veschost/)

if(DEFINED ENV{COLCON_PREFIX_PATH})
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(spdlog REQUIRED)         # Required by vescpp
  find_package(nlohmann_json REQUIRED)  # Required by vescpp

  install(
    TARGETS 
      ${PROJECT_NAME} 
      vescpp 
    EXPORT export_${PROJECT_NAME} 
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  
  install(
    DIRECTORY   
      veschost/include/
    DESTINATION include
  )

  ament_export_include_directories(
    include/
  )

  ament_export_libraries(
    ${PROJECT_NAME} 
    vescpp
  )
  ament_export_dependencies(nlohmann_json spdlog)

  ament_export_targets(
    export_${PROJECT_NAME}
  )

  ament_package()
else()
  # Lyra is used in all examples so fetch it here
  if(NOT TARGET lyra)
    find_package(lyra QUIET)
    if(NOT lyra_FOUND)
      FetchContent_Declare(lyra
        GIT_REPOSITORY  https://github.com/bfgroup/Lyra
        GIT_TAG         1.6.1
        GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
        FIND_PACKAGE_ARGS
      )
      FetchContent_MakeAvailable(lyra)
      message(STATUS "[+] Use FetchContent to get lyra 1.6.1")
    else()
      message(STATUS "[+] Use System-provided lyra ${lyra_VERSION}")
    endif()
  endif()
  add_subdirectory(bench/)
  add_subdirectory(tests/)
endif()
