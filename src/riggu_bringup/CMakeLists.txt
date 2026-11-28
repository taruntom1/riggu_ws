cmake_minimum_required(VERSION 3.8)
project(riggu_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch systemd
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # mode:=mock end to end: controllers active, /joint_states, /imu, /cmd_vel_safe -> /odom.
  # Own ROS_DOMAIN_ID so it doesn't see a sim running on the same zenoh router.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_bringup_mock.py TIMEOUT 180 ENV ROS_DOMAIN_ID=87)
endif()

ament_package()
