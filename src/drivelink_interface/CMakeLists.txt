cmake_minimum_required(VERSION 3.8)
project(drivelink_interface)

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Ensure Qt6 is findable (you may already have this in your environment)
# Provide Qt directories if not included  in the environment
list(APPEND CMAKE_PREFIX_PATH "/home/tarun/Qt/6.9.0/gcc_64/lib/cmake")

# Enable Qt automoc and autouic if needed
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON) # if using .qrc files
set(CMAKE_AUTOUIC ON) # if using .ui files (optional)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Qt6 REQUIRED COMPONENTS Core SerialPort)

find_package(yaml-cpp REQUIRED)


# Collect source and header files
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/include/drivelink_interface/*.h")

# Add the executable
add_executable(${PROJECT_NAME}_node ${SOURCES} ${HEADERS})

# Set include directories
target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include/drivelink_interface
)

# Link Qt6 and ROS2 libraries
target_link_libraries(${PROJECT_NAME}_node
  Qt6::Core
  Qt6::SerialPort
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  ament_index_cpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/../"
)


# Install the node
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
