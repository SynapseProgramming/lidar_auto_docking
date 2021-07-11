cmake_minimum_required(VERSION 3.5)
project(lidar_auto_docking)

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
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dock.action"
  "action/Undock.action"
  "msg/Initdock.msg"
   DEPENDENCIES geometry_msgs
)

add_executable(dock_coordinates src/dock_coordinates.cpp src/perception.cpp src/laser_processor.cpp src/linear_pose_filter_2d.cpp src/icp_2d.cpp src/tf2listener.cpp)
ament_target_dependencies(dock_coordinates geometry_msgs tf2 sensor_msgs rclcpp Eigen3 angles tf2_ros tf2_geometry_msgs)
#allow target to access header files defined in include
target_include_directories( dock_coordinates
 PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(dock_coordinates ${PROJECT_NAME} "rosidl_typesupport_cpp")



add_executable(auto_dock src/autodock.cpp src/controller.cpp src/perception.cpp src/laser_processor.cpp src/linear_pose_filter_2d.cpp src/icp_2d.cpp src/tf2listener.cpp)
ament_target_dependencies(auto_dock nav_msgs geometry_msgs tf2 sensor_msgs rclcpp rclcpp_action Eigen3 angles tf2_ros tf2_geometry_msgs )
target_include_directories( auto_dock
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(auto_dock ${PROJECT_NAME} "rosidl_typesupport_cpp")


ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/dock_robot.py
  scripts/undock_robot.py
  scripts/dock_saver.py
  scripts/nav2_dock.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
  action
  config
  msg
  launch
  initial_dock_pose
  DESTINATION share/${PROJECT_NAME}
)






install(TARGETS
auto_dock
dock_coordinates
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
