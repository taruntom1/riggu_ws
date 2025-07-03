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
