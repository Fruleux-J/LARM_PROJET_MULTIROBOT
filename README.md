# Installation
For that project you need ros2 iron, rviz2 and nav2.
Moreover, on the robot you need ros2 to communicate with the package
Finally, you need to download this package and colon build.

## get started
Comment le lancer si besoin
For this project you have multiple programs, the robot, usine, nav2 rviz2 programs.

## First step
Command to be on the same domain id of the robot
```
export ROS_DOMAIN_ID=<Robot_domain_id>
```

launch rviz2
```
rviz2
```
be careful for us, we used a special config for rviz2,
- Fixed Frame : map
- Topic : /map

after that you need to load the map with 

```
ros2 launch nav2_bringup localization_launch.py map:=</path/to/map.yaml> autostart:=True use_sim_time:=True
```
finally, you need to activate the navigation of nav2 with

```
ros2 launch nav2_bringup navigation_launch.py
```
Repeat this process for each robot you need to move
(don't hesitate to use the goal pose on rviz2 to know if nav2 is working or not)

## Second step
You need to start the robot program,
the purpose of this program is to reach the right area on the map depending on the point the usine program gives it.

To start you need to do : 
```
export ROS_DOMAIN_ID=<Robot_domain_id>
```
And start the program on your ros2 environment

```
ros2 run <package_name> robot
```

follow this step for each robot you want to move

## Third step
### Version 1.0 
The usine manages the generation of packages, asks the robot for the cost of the delivery, and asks the robot for the least cost to deliver it.

Command to run the usine program 

```
ros2 run multi_robot_larm usine_calibration
```

### Version 1.1
This version is for the people with the calibration program that is working
```
ros2 run <package_name> usine_calibration --input-file </path/to/points.yaml> --domain-file </path/to/ros.yaml>
```
the points.yaml contains the different areas for the delivery
the ros.yaml contains the different ROS_DOMAIN_ID of your robots

## Fourth step
To start the visualisation of the markers you need to (in your ros environment) :

```
export ROS_DOMAIN_ID=<Robot_domain_id>

ros2 run <package_name> marker_visualizer
```

## Calibration
This program is to create the points.yaml you can use for the version 1.1; for that, you need to run those commands

 ```
rviz2
ros2 run <package_name> calibration --ros-args -p output_file:=</path/to/points.yaml> -p Deposit:=True
```
And click on "Publish Point" on rviz2 to create the point and get the deposit position

```
rviz2
ros2 run <package_name> calibration --ros-args -p output_file:=</path/to/points.yaml> -p Arrival:=True
```
And click on "Publish Point" on rviz2 to create the point and get the arrival position

## Change the usine manually

If the calibration doesn't work or if you want to choose the ROS_DOMAIN_ID manually, you can change the nodes in the main()
```
nodes = [
        FactoryNode(<ROS_DOMAIN_ID>, messages, usine, 6.0),
        FactoryNode(<ROS_DOMAIN_ID>, messages, usine, 6.0),
    ]
```
and for the areas, you can change every point you have in the function 
``` 
generate_packages()
```

# How work the project
We have the class Usine that we'll use to create a node in each ros_domain_id of each robot, with that the usine can communicate with each robot.
The usine will generate the packages randomly, and send a message of type
```
Package.msg
--------------
int64 id
geometry_msgs/PoseStamped departure
geometry_msgs/PoseStamped arrival
```
for each ROS_DOMAIN_ID in the topic /start_bid
each robot with the robot program will calculate its cost for delivering this package and send a message of type
```
Bid.msg
--------------
int64 id
int64 ros_domain_id
float64 cost
```
in the topic /bid

The usine will choose the ROS_DOMAIN_ID with the least cost and send it the final Package.msg in the /win_bid

After that, the robot will add the package in its waiting_list and start to move to the first point, etc...

# Architecture

The usine program will create a node in each ROS_DOMAIN_ID and publish in topics /start_bid and win_bid, it will also subscribe to the topic /bid.
The robot will do the reverse


# Future work

For the future, there is a list of what we can do:
- Add a launch file to automate the start of the package
- The collision, every robot uses nav2 to move, so you need to add some protection to stop them before they block themselves (Ex. use a radius of protection and when two robots meet, if they are in front of each other, one of them stop and not the other)
- The algorithm of cost, for now, uses only the distance to deliver every package the robot already has + the package in the bid
- Add the calibration in the markers program; for now, you need to change the position of the areas for the markers in the program and not in a .yaml or another
- Add a system to add or delete a robot from the crew; it means use a protocol to give its presence to the Usine. Usine gives it a ROS_DOMAIN_ID, and the new robot can now start to move with the others.
