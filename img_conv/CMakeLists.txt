cmake_minimum_required(VERSION 3.8)
project(img_conv)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(img_conv REQUIRED)

# Generate service messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ImgConvert.srv"  # Ensure correct filename and lowercase
  DEPENDENCIES std_msgs
)

# Ensure dependencies are built in the correct order
ament_export_dependencies(rosidl_default_runtime)

# Add Executables
add_executable(publisher src/publisher.cpp)
add_executable(img_subscriber src/img_subscriber.cpp)
add_executable(img_service src/img_service.cpp)
add_executable(img_client src/img_client.cpp)

# Link dependencies
ament_target_dependencies(publisher rclcpp sensor_msgs std_msgs)
ament_target_dependencies(img_subscriber rclcpp sensor_msgs std_msgs OpenCV cv_bridge)
ament_target_dependencies(img_service rclcpp sensor_msgs std_msgs OpenCV cv_bridge)
ament_target_dependencies(img_client rclcpp std_msgs)
# Link the generated service header to img_service

rosidl_target_interfaces(img_service
  ${PROJECT_NAME} rosidl_typesupport_cpp)

rosidl_target_interfaces(img_client
  ${PROJECT_NAME} rosidl_typesupport_cpp)
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#   target_link_libraries(img_service ${cpp_typesupport_target})
  

# Install executables
install(TARGETS
  publisher
  img_subscriber
  img_service
  img_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
