# S

## Save map published at custom topic

```bash
# save from topic `my_custom_map` to file with name `my_map`
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -r /map:=/my_custom_map
```
