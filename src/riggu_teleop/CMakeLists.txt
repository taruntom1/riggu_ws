cmake_minimum_required(VERSION 3.8)
project(riggu_teleop NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS riggu_teleop/stop_latch.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stop_latch)
install(PROGRAMS riggu_teleop/arm_switch.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_switch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_teleop_logic test/test_teleop_logic.py)
  ament_add_pytest_test(test_teleop_config test/test_teleop_config.py)
endif()

ament_package()
