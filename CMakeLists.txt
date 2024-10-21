cmake_minimum_required(VERSION 3.8)
project(pkg_main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pkg_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(traj_action_server src/traj_action_server.cpp)
ament_target_dependencies(traj_action_server
      "rclcpp" "rclcpp_action" "sensor_msgs" "pkg_interfaces")
 
add_executable(task src/task.cpp)
ament_target_dependencies(task
      "rclcpp" "rclcpp_action" "sensor_msgs" "pkg_interfaces")
      
install(TARGETS 
       traj_action_server task
       DESTINATION lib/${PROJECT_NAME}
       )
       
 install(DIRECTORY launch
         DESTINATION share/${PROJECT_NAME})
       

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

ament_package()
