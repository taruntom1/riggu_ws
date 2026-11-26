# riggu_navigation

- **Runs on the Pi 5.** Watch the CPU: size MPPI batch/rate and costmap update rates for it.
- **Nav2 output:** Nav2 publishes `/cmd_vel_nav` to `twist_mux`, never directly to the controller.
- **Collision monitor:** its config lives in `riggu_bringup`, not here.
- **Planner and controller:** Smac 2D planner and MPPI (DiffDrive) controller. Localisation uses AMCL on a saved map. Mapping uses slam_toolbox online async.
- **Maps:** real-robot maps go to `~/riggu_maps` on the Pi and are not committed. Only the sim reference map lives in `maps/`.
- **Lidar range:** RPLIDAR C1 range limits (0.05–12 m) apply to slam_toolbox, AMCL, and costmaps.
