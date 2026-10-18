cmake_minimum_required(VERSION 3.16)
project(uav_navigation_ros VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
find_package(ament_cmake REQUIRED)
find_package(uav_navigation_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATHS /opt/tensorrt/include)
find_library(TENSORRT_LIBRARY nvinfer PATHS /opt/tensorrt/lib)
find_path(CUDA_RUNTIME_INCLUDE_DIR cuda_runtime_api.h PATHS /opt/cuda/include)
find_library(CUDA_RUNTIME_LIBRARY cudart PATHS /opt/cuda/lib)

add_library(depth_predictor STATIC src/depth_predictor.cpp)
target_compile_features(depth_predictor PUBLIC cxx_std_17)
target_include_directories(depth_predictor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_predictor PUBLIC
  uav_navigation_core::uav_navigation_core ${OpenCV_LIBS})
if(TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY AND
   CUDA_RUNTIME_INCLUDE_DIR AND CUDA_RUNTIME_LIBRARY)
  target_compile_definitions(depth_predictor PRIVATE UAV_DEPTH_TENSORRT=1)
  target_include_directories(depth_predictor PRIVATE
    ${TENSORRT_INCLUDE_DIR} ${CUDA_RUNTIME_INCLUDE_DIR})
  target_link_libraries(depth_predictor PUBLIC
    ${TENSORRT_LIBRARY} ${CUDA_RUNTIME_LIBRARY})
  # Jetson driver libraries are mounted at runtime, not in the build stage.
  target_link_options(depth_predictor INTERFACE -Wl,--allow-shlib-undefined)
endif()

add_executable(octomap_route_replanner_node src/octomap_route_replanner_node.cpp)
target_compile_features(octomap_route_replanner_node PRIVATE cxx_std_17)
target_link_libraries(octomap_route_replanner_node uav_navigation_core::uav_navigation_core)
ament_target_dependencies(octomap_route_replanner_node
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros)
set_target_properties(octomap_route_replanner_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(local_navigation_node src/local_navigation_node.cpp
  src/navigation/helpers.cpp src/navigation/configuration.cpp
  src/navigation/prior_region.cpp
  src/navigation/inputs.cpp src/navigation/control.cpp
  src/navigation/recovery.cpp src/navigation/diagnostics.cpp
  src/navigation/visualization.cpp)
target_compile_features(local_navigation_node PRIVATE cxx_std_17)
target_include_directories(local_navigation_node PRIVATE include)
target_link_libraries(local_navigation_node uav_navigation_core::uav_navigation_core)
ament_target_dependencies(local_navigation_node
  rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs diagnostic_msgs tf2_ros)
add_executable(monocular_depth_node src/monocular_depth_node.cpp)
target_compile_features(monocular_depth_node PRIVATE cxx_std_17)
target_link_libraries(monocular_depth_node depth_predictor)
ament_target_dependencies(monocular_depth_node
  rclcpp sensor_msgs diagnostic_msgs)
set_target_properties(monocular_depth_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_library(autopilot_adapter_logic STATIC src/autopilot_adapter.cpp)
target_compile_features(autopilot_adapter_logic PUBLIC cxx_std_17)
target_include_directories(autopilot_adapter_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(autopilot_adapter_node src/autopilot_adapter_node.cpp)
target_compile_features(autopilot_adapter_node PRIVATE cxx_std_17)
target_link_libraries(autopilot_adapter_node autopilot_adapter_logic)
ament_target_dependencies(autopilot_adapter_node
  rclcpp geometry_msgs diagnostic_msgs mavros_msgs)
set_target_properties(autopilot_adapter_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(odometry_tf_node src/odometry_tf_node.cpp)
target_include_directories(odometry_tf_node PRIVATE include)
target_link_libraries(odometry_tf_node uav_navigation_core::uav_navigation_core)
ament_target_dependencies(odometry_tf_node rclcpp geometry_msgs nav_msgs tf2_ros)
set(nodes local_navigation_node monocular_depth_node octomap_route_replanner_node
  odometry_tf_node autopilot_adapter_node)
foreach(node IN LISTS nodes)
  set_target_properties(${node} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  if(APPLE)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
    target_link_libraries(${node} Python3::Python)
  endif()
endforeach()
if(BUILD_TESTING)
  add_executable(test_odometry_state test/test_odometry_state.cpp)
  target_compile_features(test_odometry_state PRIVATE cxx_std_17)
  target_include_directories(test_odometry_state PRIVATE include)
  target_link_libraries(test_odometry_state uav_navigation_core::uav_navigation_core)
  ament_target_dependencies(test_odometry_state nav_msgs)
  if(APPLE)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
    target_link_libraries(test_odometry_state Python3::Python)
  endif()
  add_test(NAME angular_odometry_state COMMAND test_odometry_state)
  add_executable(test_autopilot_adapter test/test_autopilot_adapter.cpp)
  target_link_libraries(test_autopilot_adapter PRIVATE autopilot_adapter_logic)
  add_test(NAME autopilot_adapter_tests COMMAND test_autopilot_adapter)
endif()
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${nodes}
  RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_package()
