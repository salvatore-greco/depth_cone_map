cmake_minimum_required(VERSION 3.8)
project(depth_cone_map)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


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
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)

add_library(subscriber src/RosHandler.cpp)
add_library(node_composition SHARED src/DepthConeMapNode.cpp)
rclcpp_components_register_nodes(node_composition
        PLUGIN "composition::DepthConeMapNode"
        EXECUTABLE depth_executable
)

ament_export_dependencies(
        rclcpp
        #   driverless_msgs
)

# Include directory
target_include_directories(subscriber PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_include_directories(node_composition PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(subscriber PUBLIC
        rclcpp
        driverless_msgs
        sensor_msgs
        visualization_msgs
        geometry_msgs
        tf2
        tf2_ros
        message_filters
        rclcpp_components

)

ament_target_dependencies(node_composition PUBLIC
        rclcpp
        rclcpp_components
        message_filters
        driverless_msgs
        geometry_msgs
        sensor_msgs
)

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

install(
        TARGETS subscriber
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
