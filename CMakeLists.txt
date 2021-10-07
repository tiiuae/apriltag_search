cmake_minimum_required(VERSION 3.5)
project(apriltag_search)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(fog_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "action/Search.action"
        DEPENDENCIES builtin_interfaces std_msgs # std_srvs
        )

add_library(${PROJECT_NAME}_server SHARED src/apriltag_search_server.cpp)
target_include_directories(${PROJECT_NAME}_server PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME}_server
        PRIVATE "CPP_BUILDING_DLL")

ament_target_dependencies(${PROJECT_NAME}_server
        "rclcpp"
        "rclcpp_action"
        "rclcpp_components"
        fog_msgs
        sensor_msgs
        )

rosidl_target_interfaces(${PROJECT_NAME}_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

rclcpp_components_register_node(${PROJECT_NAME}_server PLUGIN "apriltag_search::ApriltagSearchServer" EXECUTABLE ${PROJECT_NAME}_server_node)

install(TARGETS
        ${PROJECT_NAME}_server
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
