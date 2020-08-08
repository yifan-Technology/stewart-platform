cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

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
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(cameraanalyse src/Camera.cpp)
ament_target_dependencies(cameraanalyse rclcpp geometry_msgs)
add_executable(controller src/Controller.cpp)
ament_target_dependencies(controller rclcpp geometry_msgs)
add_executable(motor src/Motor.cpp)
ament_target_dependencies(motor rclcpp geometry_msgs)
install(TARGETS
  cameraanalyse
  controller
  motor
  DESTINATION lib/${PROJECT_NAME})


ament_package()
