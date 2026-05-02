cmake_minimum_required(VERSION 3.8)
project(depth_cone_map)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

include(FetchContent)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(driverless_msgs REQUIRED) # For Bounding boxes and marker arrays
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # FOR DEPTH IMAGE
find_package(visualization_msgs REQUIRED) # For publishing cone position (to be compatible with traj)
# To make transformation from camera to world frame
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#cv bridge to transform image in cv::Mat
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)

# including simdjson for bounding boxes message parsing
# source: Using simdjson as a CMake dependency https://simdjson.github.io/simdjson/md_doc_2basics.html
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
FetchContent_Declare(
        simdjson
        GIT_REPOSITORY https://github.com/simdjson/simdjson.git
        GIT_TAG  tags/v4.6.1
        GIT_SHALLOW TRUE)

FetchContent_MakeAvailable(simdjson)
add_library(source src/ImageProcessor.cpp src/RosHandler.cpp src/ImageTransformer.cpp)

add_library(node_composition SHARED src/DepthConeMapNode.cpp)
rclcpp_components_register_node(
  node_composition
 PLUGIN "DepthConeMapNode"
 EXECUTABLE node_executable)

ament_export_dependencies(
        rclcpp
        #   driverless_msgs
)

# Include directory
target_include_directories(node_composition PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_include_directories(source PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(source PUBLIC simdjson)
ament_target_dependencies(source PUBLIC
        rclcpp
        driverless_msgs
        geometry_msgs
        sensor_msgs
        cv_bridge
        OpenCV
        driverless_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        message_filters
        rclcpp_components
)

target_link_libraries(node_composition PUBLIC simdjson source)
ament_target_dependencies(node_composition PUBLIC
        rclcpp
        rclcpp_components
        message_filters
        driverless_msgs
        geometry_msgs
        sensor_msgs
        visualization_msgs
        cv_bridge
        tf2
        tf2_ros
        tf2_geometry_msgs
)


# add_executable(node_executable src/main.cpp)
# target_link_libraries(node_executable PUBLIC simdjson node_composition source)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()
install(TARGETS node_composition
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  node_executable
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
