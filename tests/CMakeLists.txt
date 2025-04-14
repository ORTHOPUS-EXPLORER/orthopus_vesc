cmake_minimum_required(VERSION 3.22)
project(VESCHostTests)

# Piggy-bakc on VESCpp tests macros :) 
include(${CMAKE_CURRENT_LIST_DIR}/../lib/vescpp/tests/cmake/vescpp_test.cmake)

add_test_target(NAME stats       
                SRCS stats.cpp       
                LIBRARIES vescpp
               )
add_test_target(NAME veschost_all          
                SRCS  all.cpp                         
                ALL
               )
if(NOT TARGET check)
  add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} DEPENDS ${all_tests})
endif()
