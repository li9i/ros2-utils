# `xyz_to_ply_transframer`

Transform a `.xyz` file expressed in one frame to a `.xyz` file expressed in another frame.

- step 1: Inspect `launch/xyz_to_ply_transframer.launch.xml` and set the values of args.

- step 2: Launch the node

  ```bash
  ros2 launch xyz_to_ply_transframer xyz_to_ply_transframer.launch.xml
  ```

- step 3 (optional): If your files are named sequentially (e.g. `Sampled_Mesh_1.xyz`, `Sampled_Mesh_2.xyz`, ...) then you may set `input_file_prefix` and `input_file_pattern` so that when you

  ```bash
  ros2 param set /xyz_to_ply_transframer input_file_id "'3'"
  ```

  `Sampled_Mesh_3.xyz` is selected as the input `.xyz` file

- step 4: Simply call the node for work

  ```bash
  ros2 service call /xyz_to_ply_transframer/execution/enable std_srvs/srv/Trigger
  ```
