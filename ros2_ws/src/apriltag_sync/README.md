# `apriltag_sync`

Synchronise one image topic of type `sensor_msgs/Image` and one camera info topic of type `sensor_msgs/CameraInfo`, possibly because you need them to detect Apriltags.

- step 1: Inspect `launch/apriltag_sync.launch.xml` and set the values of args.

- step 2: Launch the node

  ```bash
  ros2 launch apriltag_sync apriltag_sync.launch.xml
  ```
