```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/glove_box_dev/install/setup.bash
```

To run it, 

```bash
ros2 launch mdx_ethercat mdx.launch.py
ros2 run mdx_ethercat read_test
ros2 run mdx_ethercat pp_test
```