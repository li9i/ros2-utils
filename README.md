### `ros2-utils`: A small collection of ROS 2 Humble utility packages

- [`apriltag_hitch_estimation`](ros2_ws/src/apriltag_hitch_estimation/README.md) Need to estimate the 6DOF transform between a RGB camera and an object in real time? Or more specifically between a mobile base and a cart hitched to it? Print an arbitrary array of April tags, launch this node, and you are good to go

- [`static_tf_republisher`](ros2_ws/src/static_tf_republisher/README.md) ROS 1 cannot look up static transforms published from ROS 2 nodes. Use this package to republish select transforms from `/tf_static` to `/tf` so that ROS 1 nodes have access to them

- [`ply_to_xyz_transframer`](ros2_ws/src/ply_to_xyz_transframer/README.md) Transform a `.ply` file expressed in one frame of reference to a `.xyz` file expressed in another

- [`xyz_to_ply_transframer`](ros2_ws/src/xyz_to_ply_transframer/README.md) Transform a `.xyz` file expressed in one frame of reference to a `.ply` file expressed in another

### Build

```bash
cd docker
docker compose build
```

### Deploy

```bash
cd docker
docker compose up -d
docker exec -it -u humble ros2_utils bash
```

### Run

```bash
ros2 launch ...
```
