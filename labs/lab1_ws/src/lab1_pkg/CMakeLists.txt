cmake_minimum_required(VERSION 3.8)
project(lab1_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Python package
ament_python_install_package(${PROJECT_NAME})

# C++ executables
add_executable(talker_cpp src/talker.cpp)
ament_target_dependencies(talker_cpp rclcpp ackermann_msgs)

add_executable(relay_cpp src/relay.cpp)
ament_target_dependencies(relay_cpp rclcpp ackermann_msgs)

# Install C++ executables
install(TARGETS
  talker_cpp
  relay_cpp
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 