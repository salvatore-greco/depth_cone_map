cmake_minimum_required(VERSION 3.8)
project(depth_cone_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # FOR DEPTH IMAGE
find_package(visualization_msgs REQUIRED) # For publishing cone position (to be compatible with traj)
# To make transformation from camera to world frame
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(${PROJECT_NAME}_node ${PROJECT_NAME}_node.cpp)

ament_export_dependencies(
  rclcpp
)

target_link_libraries(${PROJECT_NAME}_node PUBLIC 
rclcpp:rclcpp 
driverless_msgs::driverless_msgs 
sensor_msgs::sensor_msgs 
visualization_msgs::visualization_msgs
tf2::tf2
tf2_ros::tf2_ros
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
