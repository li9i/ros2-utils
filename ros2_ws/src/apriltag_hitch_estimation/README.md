# `apriltag_hitch_estimation`

Estimate the 6DOF transform between a RGB camera and an array of April tags, or that between a mobile base and a cart hitched to the base.

Set your specific frames, topic, params, etc in `config/params.yaml`. Then inspect launchers in `launch/`.

Estimate the angle between the mobile base and the trailer with

```bash
ros2 launch apriltag_hitch_estimation apriltag_hitch_estimation.launch.xml
```

Instead, if you wish to estimate only the full 6DOF of the April tag array with respect to the RGB camera, comment out the last include, namely

```xml
<include file="$(find-pkg-share apriltag_hitch_estimation)/launch/hitch_joint_estimation.launch.xml"/>
```
