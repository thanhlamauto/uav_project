cmake_minimum_required(VERSION 3.16)
project(uav_navigation_core VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OCTOMAP REQUIRED IMPORTED_TARGET octomap)

add_library(${PROJECT_NAME} SHARED
  src/command_conditioner.cpp
  src/metric_depth.cpp
  src/octomap_planner.cpp
  src/static_octomap_planner.cpp
  src/mppi/cost_evaluator.cpp
  src/mppi/dynamics.cpp
  src/mppi/optimizer.cpp
  src/mppi/path_reference.cpp
  src/mppi/rollout.cpp
  src/mppi/route_blockage.cpp
  src/stabilization.cpp
  src/trajectory_safety_checker.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC PkgConfig::OCTOMAP)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR})

if(BUILD_TESTING)
  add_executable(test_prior_guard test/test_prior_guard.cpp)
  target_link_libraries(test_prior_guard PRIVATE ${PROJECT_NAME})
  add_test(NAME prior_region_safety COMMAND test_prior_guard)
  add_executable(test_static_octomap_planner test/test_static_octomap_planner.cpp)
  target_link_libraries(test_static_octomap_planner PRIVATE ${PROJECT_NAME})
  if(UAV_STATIC_PRIOR_TEST_FILE)
    add_test(NAME prior_forest_parity
      COMMAND test_prior_guard "${UAV_STATIC_PRIOR_TEST_FILE}")
    add_test(NAME static_octomap_parity
      COMMAND test_static_octomap_planner "${UAV_STATIC_PRIOR_TEST_FILE}")
  endif()
  add_executable(test_yaw_state test/test_yaw_state.cpp)
  target_link_libraries(test_yaw_state PRIVATE ${PROJECT_NAME})
  add_test(NAME measured_yaw_state COMMAND test_yaw_state)
  add_executable(test_runtime test/test_runtime.cpp)
  target_link_libraries(test_runtime PRIVATE ${PROJECT_NAME})
  add_test(NAME runtime_geometry_and_safety COMMAND test_runtime)
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
# Export through ament when ROS 2 is available. The library itself remains
# independent of ROS, Gazebo and SDF.
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_package(CONFIG_EXTRAS cmake/octomap_dependencies.cmake)
else()
  install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
  configure_package_config_file(
    cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
endif()
