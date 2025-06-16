# router start
    -   ros2 run rmw_zenoh_cpp rmw_zenohd
# remote zenoh router
    -   export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.0.3:7447", "tcp/192.168.0.4:7447"]'
