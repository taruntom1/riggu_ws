# router start
    -   ros2 run rmw_zenoh_cpp rmw_zenohd
# remote zenoh router
    -   export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/100.107.192.97:7447"]'

# Build for ARM using Docker Buildx
    docker buildx build \
    --platform linux/arm64 \
    -t my-qt-ros-app-builder \
    --load .

docker create --name temp-ros-build my-qt-ros-app-builder
docker cp temp-ros-build:/ros2_ws/install ./install
docker cp temp-ros-build:/ros2_ws/build ./build
docker rm temp-ros-build


ros2 run twist_mux twist_mux --ros-args --params-file ./src/riggu_bringup/config/twist_mux_config.yaml -r cmd_vel_out:=diff_drive/cmd_vel
ros2 launch nav2_bringup navigation_launch.py  use_sim_time:=false params_file:=/riggu_ws/src/riggu_bringup/config/nav2_params.yaml 
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser