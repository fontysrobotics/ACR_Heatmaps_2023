<div align="center">
<h1 align="center">Heat Map package developed on the Turtlebot4 for Cleaning Robots</h1>
Make heat maps of the positional data of the Turtlebot 4
<img width="70%" src=turtlebot4_heat_map/resources/Heat_Map_example.png>
</div>

---

## About
This is a mechatronics internship project issued by the Adaptive Robotics group at Fontys Nexus in Eindhoven. The robot development used a Turtlebot 4 instead of an actual cleaning robot, therefore the heat map package is semi-integrated with the turtlebot 4 packages. This package tracks the robot's positional data and records it over months to create: a live map, a total coverage map, and a most recent covered map.

The path planner and map generator is integrated within the ROS2 framework, while the other scripts to make the maps run solely on Python. The maps are packages in a very basic UI, recommended to restart when trying to remake a map. For more information on the project, refer to the report 'Cleaning Robot Heat Maps'.

Currently lacking live updates with the live map in the UI.
To track positional data, a subscriber listens to messages from the `amcl_pose` topic in the map generator. If a different robot should be used in the future, ensure that the topic name is similar or change the listener's subscribed topic. 

---

## Credits
Contributor: Shananda Surjadi
### Reference Material
- <a href="https://docs.ros.org/en/humble/Tutorials.html" target="_blank"> ROS2 Tutorials </a>
- <a href="https://turtlebot.github.io/turtlebot4-user-manual/overview/resources.html" target="_blank">  Turtlebot4 User Manual</a>
- <a href="http://classic.gazebosim.org/tutorials" target="_blank"> Gazebo docs </a>

---

## Startup Guide
1. Must have linux OS installed
   
2. Clone and source the packages below into the local workspace or install from apt packages:
- <a href="https://github.com/turtlebot/turtlebot4" target="_blank"> Turtlebot4 </a>
- <a href="https://github.com/turtlebot/turtlebot4_robot" target="_blank"> Turtlebot4 robot </a>
- <a href="https://github.com/turtlebot/turtlebot4_desktop" target="_blank"> Turtlebot4 desktop </a>

  Use any preferred discovery method with the Turtlebot

3. Clone and source this Github package into the workspace
   
4. make a directory called `maps` in /home/`user`

## Running the Program
1. Make a map using SLAM using:
```shell
ros2 launch turtlebot4_navigation slam.launch.py
```
   and save the map in the `maps` directory
  
2. In a new terminal, launch localization, nav2 and rviz using:
```shell
ros2 launch turtlebot4_navigation localization.launch.py map:=<map_name>;
ros2 launch turtlebot4_navigation nav2.launch.py;
ros2 launch turtlebot4_viz view_robot.launch.py
```

3. Open a new terminal and run the navigator using:
```shell
ros2 run turtlebot4_heat_map path_plan --ros-args -p load_file:=<year>-<month>-<day>
```
  specify a date file to load a saved path to run or leave it empty to create a new configuration.
  To create the goal poses, use the 'SetPoseEstimate' tool in Rviz and create as many poses as you want.
  
4. In a new terminal, run the map generator node using the command:
```shell
ros2 run turtlebot4_heat_map map_generator --ros-args -p map_name:=<map_name>
```
  always specify a map name to configure the heat maps correctly. To know that this node is running correctly, the terminal will output 'node is spinning'. If the map is delayed, it is likely because this node is not spinning immediately.

The other map scripts can be called directly in Python, and is also called directly in the gui if not.
