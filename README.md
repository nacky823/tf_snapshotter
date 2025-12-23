# tf_snapshotter

## Usage

Build:

```bash
colcon build --symlink-install --packages-select tf_snapshotter
```

Run with params(ex.):

```bash
ros2 run tf_snapshotter tf_snapshotter_node
```
```bash
ros2 run tf_snapshotter tf_snapshotter_node --ros-args --params-file $HOME/rdd_ws/src/tf_snapshotter/config/tf_snapshotter.yaml
```

Capture home:

```bash
ros2 service call /snapshot std_srvs/srv/Trigger "{}"
```

Verify behavior(not required):

```bash
ros2 run tf2_ros tf2_echo map target_frame
```

Publish condition topic (FINISH must be held for a while):

```bash
ros2 topic pub -r 10 /state/detector std_msgs/msg/String "{data: FINISH}"
```

Stop condition (FINISH breaks for a while):

```bash
ros2 topic pub -r 10 /state/detector std_msgs/msg/String "{data: RUNNING}"
```
