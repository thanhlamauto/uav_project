cmake_minimum_required(VERSION 3.16)
project(uav_octomap_tools LANGUAGES CXX)
include(CTest)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OCTOMAP REQUIRED IMPORTED_TARGET octomap)
find_package(tinyxml2 CONFIG REQUIRED)

add_library(sdf_geometry STATIC sdf_geometry.cpp)
target_compile_features(sdf_geometry PUBLIC cxx_std_17)
target_include_directories(sdf_geometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(sdf_geometry PUBLIC tinyxml2::tinyxml2)
add_executable(build_octomap_world build_octomap_world.cpp)
target_link_libraries(build_octomap_world PRIVATE sdf_geometry PkgConfig::OCTOMAP)

if(BUILD_TESTING)
  foreach(mode IN ITEMS full partial)
    set(fraction 0)
    if(mode STREQUAL "partial")
      set(fraction 1)
    endif()
    add_test(NAME octomap_export_${mode} COMMAND build_octomap_world
      --world ${CMAKE_CURRENT_SOURCE_DIR}/test/world.sdf
      --output ${CMAKE_CURRENT_BINARY_DIR}/test/${mode}
      --min -4 -4 0 --max 8 4 6 --resolution 0.5
      --hidden-fraction ${fraction} --seed 17)
    set_tests_properties(octomap_export_${mode} PROPERTIES FIXTURES_SETUP ${mode}_map)
  endforeach()
  add_executable(test_octomap_preparation test/test_octomap_preparation.cpp)
  target_link_libraries(test_octomap_preparation PRIVATE sdf_geometry PkgConfig::OCTOMAP)
  add_test(NAME octomap_geometry_and_outputs COMMAND test_octomap_preparation
    ${CMAKE_CURRENT_SOURCE_DIR}/test/world.sdf ${CMAKE_CURRENT_BINARY_DIR}/test)
  set_tests_properties(octomap_geometry_and_outputs PROPERTIES FIXTURES_REQUIRED "full_map;partial_map")
endif()
