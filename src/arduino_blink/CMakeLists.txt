cmake_minimum_required(VERSION 3.8)
project(arduino_blink)



# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)  # For serial communication

# Create the library
add_library(${PROJECT_NAME} SHARED
  src/blink.cpp
)

add_library(serial_pic STATIC IMPORTED)
set_target_properties(serial_pic PROPERTIES
  IMPORTED_LOCATION /path/to/libserial.a
  INTERFACE_INCLUDE_DIRECTORIES /path/to/serial/include
)
# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



# Plugin declaration (FIXED: use project name and correct XML file)
pluginlib_export_plugin_description_file(${PROJECT_NAME} 

plugins/arduino_hardware.xml

)

# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins/arduino_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)
# Link dependencies using ament_target_dependencies
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  serial
  pluginlib
)
# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  serial
  pluginlib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()