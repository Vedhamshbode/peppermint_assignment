cmake_minimum_required(VERSION 3.8)
project(bbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config urdf launch worlds meshes
  DESTINATION share/${PROJECT_NAME}
)
add_executable(pose_navigator src/pose_navigator.cpp)
ament_target_dependencies(pose_navigator rclcpp rclcpp_action nav2_msgs)

install(TARGETS
  pose_navigator
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS
  src/republisher.py
  src/obj_publisher.py
  src/image_subscriber.py
  src/voice_client.py
  src/voice_service.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
