cmake_minimum_required(VERSION 3.16)
project(jikl_fc01_system)
# Executables live in lib/<package>; generated typesupport and its dlopen
# backends live in lib. Keep the installed bundle self-contained after relocation.
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/..")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisionState.msg"
  "msg/NavigationCommand.msg"
  "msg/VelocityCommand.msg"
  "msg/VisionHealth.msg"
  "msg/AttitudeThrustPreview.msg"
  DEPENDENCIES builtin_interfaces)
rosidl_get_typesupport_target(fc01_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
add_executable(fc01_system_node src/fc01_system_node.cpp)
add_executable(fc01_config_check src/fc01_config_check.cpp)
target_compile_features(fc01_config_check PUBLIC cxx_std_17)
target_compile_options(fc01_config_check PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(fc01_config_check PRIVATE include)
ament_target_dependencies(fc01_config_check rclcpp px4_msgs)
target_compile_features(fc01_system_node PUBLIC cxx_std_17)
target_compile_options(fc01_system_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(fc01_system_node PRIVATE include)
ament_target_dependencies(fc01_system_node rclcpp px4_msgs)
target_link_libraries(fc01_system_node Threads::Threads "${fc01_typesupport}")
install(TARGETS fc01_system_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS fc01_config_check DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/vision_odometry_adapter.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/velocity_command_diagnostic.py"
  DESTINATION lib/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/vision_state_validation.py"
  DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
