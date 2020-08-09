cmake_minimum_required(VERSION 3.5)
project(stewart2)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(InverseKinematicsStewart2 src/InverseKinematicsStewart2.cpp)
ament_target_dependencies(InverseKinematicsStewart2 rclcpp std_msgs geometry_msgs Eigen3 trajectory_msgs builtin_interfaces sensor_msgs) 

add_executable(ForwardKinematicsStewart2 src/ForwardKinematicsStewart2.cpp)
ament_target_dependencies(ForwardKinematicsStewart2 rclcpp std_msgs geometry_msgs Eigen3 trajectory_msgs builtin_interfaces sensor_msgs) 

add_executable(ImuAnalyse src/ImuAnalyse.cpp)
ament_target_dependencies(ImuAnalyse rclcpp std_msgs geometry_msgs Eigen3 trajectory_msgs builtin_interfaces sensor_msgs) 

add_executable(gazeboBillotController2 src/gazeboBillotController2.cpp)
ament_target_dependencies(gazeboBillotController2 rclcpp std_msgs geometry_msgs Eigen3 trajectory_msgs builtin_interfaces sensor_msgs) 

install(TARGETS
  ImuAnalyse
  InverseKinematicsStewart2
  ForwardKinematicsStewart2
  gazeboBillotController2
  DESTINATION lib/${PROJECT_NAME})


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
