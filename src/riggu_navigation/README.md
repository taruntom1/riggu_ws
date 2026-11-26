# riggu_navigation

Mapping, localisation, and Nav2 for riggu. *Planned — Phases 7–8.*

| File | Purpose |
|---|---|
| `launch/slam.launch.py` | slam_toolbox online async |
| `launch/localization.launch.py` | map_server + AMCL. Arg: `map` |
| `launch/navigation.launch.py` | Localisation + Nav2. Args: `map`, `slam:=true` (navigate while mapping) |
| `config/slam_toolbox.yaml` | 5 cm resolution, C1 range 0.05–12 m |
| `config/nav2_params.yaml` | AMCL (diff), Smac 2D, MPPI (DiffDrive, ~20 Hz), velocity smoother, costmaps from `/scan` |
| `scripts/waypoints.py` | Runs a YAML list of poses via `FollowWaypoints`. Option: `--loop` |
| `maps/` | Sim reference map only. Real maps live in `~/riggu_maps` on the Pi |
| `rviz/` | Mapping and navigation views |

Nav2 publishes `/cmd_vel_nav`, which reaches the controller only through `twist_mux` and `collision_monitor` (see `riggu_bringup`).

```bash
ros2 launch riggu_navigation slam.launch.py
scripts/save_map.sh <name>
ros2 launch riggu_navigation navigation.launch.py map:=$HOME/riggu_maps/<name>.yaml
```
