To run:
```bash
cd ~/Desktop/webots_25a_ws/SeniorDesign_4a
source ~/Desktop/webots_25a_ws/SeniorDesign_4a/ros2/sim_env.sh
ros2 launch bringup octomap_mapping.launch.py map:="PATH_TO_OCTOMAP_WORLD.bt"
```
In my case:
```bash
ros2 launch bringup octomap_mapping.launch.py map:=/home/ianho/break_room_floor.bt
```
To save map:
```bash
ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:="PATH_TO_OCTOMAP_WORLD.bt"
```
To get waypoint, use RViz or:
```bash
ros2 service call /plan_path nav_msgs/srv/GetPlan \
  "{start: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}}},
    goal:  {header: {frame_id: map}, pose: {position: {x: 3.0, y: -2.0, z: 0.0}}}}"
```
Or for the robots current pos to any other pos:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.0, y: -2.0, z: 0.0}}}"
```
