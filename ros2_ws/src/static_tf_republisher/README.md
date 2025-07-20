# `static_tf_republisher`

Republish select transforms from `/tf_static` to `/tf` so that ROS 1 nodes may access them

Set the transforms between frames in `config/frames.yaml` and then

```bash
ros2 launch static_tf_republisher static_tf_republisher.launch.xml
```
