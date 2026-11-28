# riggu_navigation

- **No `nav2_bringup` dependency:** it pulls Gazebo onto the Pi (through the Nav2 TurtleBot sims). Write our own launch files from `nav2_*` packages.
- **Runs on the Pi 5.** Watch the CPU: size MPPI batch/rate and costmap update rates for it.
- **Nav2 output:** Nav2 publishes `/cmd_vel_nav` to `twist_mux`, never directly to the controller.
- **Collision monitor:** its config lives in `riggu_bringup`, not here.
- **Planner and controller:** Smac 2D planner and MPPI (DiffDrive) controller. Localisation uses AMCL on a saved map. Mapping uses slam_toolbox online async.
- **Maps:** real-robot maps go to `~/riggu_maps` on the Pi and are not committed. Only the sim reference map lives in `maps/`.
- **Lidar range:** RPLIDAR C1 range limits (0.05–12 m) apply to slam_toolbox, AMCL, and costmaps. The launches read them from `riggu_description/config/robot.yaml` (`lidar.range_*`); don't repeat them in the yaml files.
- **Lifecycle:** every node runs under a nav2 `lifecycle_manager` (autostart, bond): `lifecycle_manager_slam`, `lifecycle_manager_localization`. slam_toolbox gets `use_lifecycle_manager: true`.
- **Frames:** these launches own only `map → odom`; `odom → base_footprint` stays with the EKF (`riggu_bringup`). Never run slam_toolbox and AMCL together.
- **Sim reference map** (`maps/house.*`): its origin is the sim spawn (world `2.5 -1.5`, yaw 0). Regenerate it if `riggu_gazebo/worlds/house.sdf` changes; `test_localization_mock.py` raycasts its scans from it.
- **slam_toolbox shutdown:** Jazzy 2.8 aborts (exit −6, `RCLError` in teardown) on SIGINT; the launch test allows −6 for that process only.
