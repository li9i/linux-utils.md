# E

## Echo field of message

```bash
ros2 topic echo --once /stereo/right/image_rect --field header.stamp
```

# P

## Publish static transform on the fly

```bash
ros2 run tf2_ros static_transform_publisher X Y Z ROLL PITCH YAW PARENT_FRAME CHILD_FRAME
```

# S

## Save map published at custom topic

```bash
# save from topic `my_custom_map` to file with name `my_map`
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -r /map:=/my_custom_map
```
