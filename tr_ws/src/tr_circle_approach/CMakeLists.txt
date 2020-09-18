cmake_minimum_required(VERSION 3.5)
project(tr_circle_approach)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)  
find_package(image_transport REQUIRED)  
find_package(cv_bridge REQUIRED)  
find_package(sensor_msgs)
find_package(OpenCV REQUIRED)  
find_package(tr_pipeline_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(tr_circle_approach SHARED
  src/circle_detector.cpp
  src/circle_pose_estimator.cpp)
rclcpp_components_register_nodes(tr_circle_approach "circle_approach::CircleDetector")
rclcpp_components_register_nodes(tr_circle_approach "circle_approach::CirclePoseEstimator")
target_compile_definitions(tr_circle_approach
  PRIVATE "tr_circle_approach_CPP_BUILDING_DLL")
ament_target_dependencies(tr_circle_approach
  "rclcpp"
  "rclcpp_components"
  "image_transport"
  "cv_bridge"
  "sensor_msgs"
  "OpenCV"
  "tr_pipeline_interfaces"
  "std_msgs"
  "geometry_msgs")

install(TARGETS
  tr_circle_approach
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
