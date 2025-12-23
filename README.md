# tf_snapshotter

## Usage

Build:

```bash
colcon build --packages-select tf_snapshotter
```

Run with params:

```bash
ros2 run tf_snapshotter tf_snapshotter_node --ros-args --params-file $HOME/rdd_ws/src/tf_snapshotter/config/tf_snapshotter.yaml
```

Capture home:

```bash
ros2 service call /snapshot std_srvs/srv/Trigger "{}"
```

Verify behavior:

```bash
ros2 run tf2_ros tf2_echo map target_frame
```

Publish condition topics (both must be true):

```bash
ros2 topic pub /target_tracker/nav_state std_msgs/msg/String "{data: finish}" -1
ros2 topic pub /state/detector std_msgs/msg/String "{data: FINISH}" -1
```

Stop condition (either topic changes):

```bash
ros2 topic pub /target_tracker/nav_state std_msgs/msg/String "{data: running}" -1
```
