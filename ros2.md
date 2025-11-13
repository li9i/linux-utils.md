# E

## Echo field of message

```bash
ros2 topic echo --once /stereo/right/image_rect --field header.stamp
```

# M

## Migrate signing key

```bash
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo rm /etc/apt/sources.list.d/ros2.list
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
```

source: https://discourse.openrobotics.org/t/ros-signing-key-migration-guide/43937#p-93537-how-do-i-migrate-after-june-1st-4

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

or, thoroughly:

```bash
ros2 run nav2_map_server map_saver_cli -f $MAP_SAVE_FILEPATH -t MAP_TOPIC_TO_SAVE --occ 0.65 --free 0.196 --ros-args -p map_subscribe_transient_local:=true
```

# T

## Throttle ros2 topic echo frequency

Echo every `10`th message in topic `/topic_name`

```bash
ros2 topic echo /topic_name | awk 'BEGIN{RS="---\n"} NR % 10 == 1 {print; printf "---\n"}'
```
