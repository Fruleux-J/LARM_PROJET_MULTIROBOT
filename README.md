## How to nav2 for mouvement

Command to be on the same domain id of the robot
```
export ROS_DOMAIN_ID=<Robot_domain_id>
```

launch rviz2
```
rviz2
```
becareful for us, we used a special config for rviz2,
Fixed Frame : map
Topic : 
- Map

after that you need to load the map with 

```
ros2 launch nav2_bringup localization_launch.py map:=</path/to/map.yaml>autostart:=True use_sim_time:=True
```
finally, you need to activate the navigation of nav2 with

```
ros2 launch nav2_bringup navigation_launch.py
```
Now, you can use the topic /goal_pose to give a position to the robot to move.