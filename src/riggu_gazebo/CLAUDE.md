# riggu_gazebo

- Use Gazebo **Harmonic** (gz-sim 8) with Jazzy `ros_gz` and `gz_ros2_control`. No Gazebo Classic APIs or plugins.
- **No sim-only behaviour configs.** Controllers, EKF, and Nav2 params come from `riggu_bringup` and `riggu_navigation`.
- **Sensors match the real hardware:**
  - Lidar: C1, 10 Hz, 0.72° resolution, 0.05–12 m range, with noise.
  - IMU: noise and bias.
  - Specs are defined once, in the description's sim sensor xacro.
- `sim.launch.py` supports `headless:=true` for CI. Bridge only `/clock` and `/scan`; the IMU comes through ros2_control.
- **World:** `worlds/house.sdf` is hand-written primitives (no Fuel). Keep doorways ≥ 0.8 m and walls above the lidar plane; Phases 7–8 map and navigate in it. Furniture must stay clear of the default spawn `2.5 -1.5`.
- **Headless checks:** `headless:=true` runs `gz sim -s --headless-rendering`; verify by reading topics, TF and `gz model -m riggu -p` (ground truth).
