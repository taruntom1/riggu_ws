cmake_minimum_required(VERSION 3.8)
project(twist_stamper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create twist_stamper executable
add_executable(twist_stamper src/twist_stamper.cpp)
target_link_libraries(twist_stamper PUBLIC
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

# Create twist_unstamper executable
add_executable(twist_unstamper src/twist_unstamper.cpp)
target_link_libraries(twist_unstamper PUBLIC
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

# Install executables
install(TARGETS
  twist_stamper
  twist_unstamper
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
