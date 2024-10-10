cmake_minimum_required(VERSION 3.8)
project(teleoperation_dual_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

#find_library(SDL2 NAMES SDL2 PATHS /usr/lib)


#teleoperation_dual_control
add_library(teleoperation_dual_control STATIC
        src/teleoperation_dual_control.cpp
)
ament_target_dependencies(
        teleoperation_dual_control
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
)
install(TARGETS teleoperation_dual_control
        DESTINATION lib/${PROJECT_NAME})


#teleoperation_dual_control_keyboard
add_executable(teleoperation_dual_control_keyboard src/teleoperation_dual_control_keyboard.cpp)
ament_target_dependencies(
        teleoperation_dual_control_keyboard
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
)
#target_link_directories(teleoperation_dual_control_keyboard SDL2)
target_link_libraries(teleoperation_dual_control_keyboard SDL2)
install(TARGETS teleoperation_dual_control_keyboard
        DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
