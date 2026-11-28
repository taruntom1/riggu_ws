cmake_minimum_required(VERSION 3.8)
project(riggu_navigation NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_navigation_config test/test_navigation_config.py)

  # mock bring-up + a fake /scan: slam_toolbox maps, AMCL localises on the sim map.
  # Own ROS_DOMAIN_ID (see riggu_bringup).
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_slam_mock.py TIMEOUT 180 ENV ROS_DOMAIN_ID=88)
  add_launch_test(test/test_localization_mock.py TIMEOUT 180 ENV ROS_DOMAIN_ID=89)
endif()

ament_package()
