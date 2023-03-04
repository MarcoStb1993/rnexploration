# RSB NBV Exploration Plugins

This package bundles plugins for the RSB NBV Exploration package that can be used to run the package together with the [Robot Statemachine (RSM) package](http://wiki.ros.org/robot_statemachine) and the [ROS navigation stack](http://wiki.ros.org/navigation).

Alternatively, an interface to [AEDE](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)'s terrain analysis and local planner is provided. It forwards the path constructed for the RneGlobalPlanner directly to [AEDE](https://github.com/HongbiaoZ/autonomous_exploration_development_environment) local planner which follows it waypoint by waypoint. Compared to the ROS navigation stack, [AEDE](https://github.com/HongbiaoZ/autonomous_exploration_development_environment) is considerably faster in reaching new goals and switching between goals but sacrifices the ability to target a desired orientation at a goal. Therefore, it is only suited for sensor configurations with an all-around FoV.

Furthermore, this package includes convenient configuration, URDF and launch-files to start a simulation with RNE straight away.

## RSM plugin

To interface the RSM, a plugin state called RnExplorationState was implemented following the [RSM's tutorial for writing a plugin state](http://wiki.ros.org/robot_statemachine/Tutorials/WritingAPluginState). This plugin state requests the current goal from the GraphConstructor class and sets it as the next navigation goal, then triggers a transition to the navigation state.

Furthermore, a node to integrate RNE into the RSM framework was written which is called RneServiceProviderNode. Basically, it receives the status of ROS navigation and forwards to RNE if a goal was reached or reaching it failed. If the exploration mode is set to *interrupt* it also checks if a goal became obsolete and tells the navigation to abort it.

### RneServiceProviderNode

This node handles the data transition between RSM and RNE because the plugin states are volatile and not able to store data. 

#### Published Topics

**goalObsolete** ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))  
Information if the current goal is still viable (only active if exploration mode is set to *interrupt*)

#### Subscribed Topics

**bestAndCurrentGoal** ([rsb_nbv_exploration_msgs/BestAndCurrentNode](../rsb_nbv_exploration_msgs/msg/BestAndCurrentNode.msg))  
Current exploration goal and node with best reward function

**explorationMode** ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))  
RSM's currently set exploration mode (true: interrupt, false: finish)

**explorationGoalStatus** ([rsm_msgs/GoalStatus](http://docs.ros.org/en/kinetic/api/rsm_msgs/html/msg/GoalStatus.html))  
RSM's currently active goal's status and pose

**stateInfo** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))  
RSM's currently active state info text

#### Parameters

**~update_frequency** (float, default: 20)  
Update rate in Hz

## Navigation plugin

A global planner which uses the edges between the robot and the goal node along the RRG as a path was implemented following the [tutorial provided by navigation](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS). It retrieves the path from the GraphConstructor class if exploration is running, if not, the standard global planner is used.

## Starting RNE

This package includes configurations and URDFs to launch gazebo based simulations of RNE deploying a [Clearpath Robotics Husky](http://wiki.ros.org/Robots/Husky) robot which simulation is available as a [ROS package](http://wiki.ros.org/husky_simulator?distro=melodic) and is required to run this launch file. Also, a simulation for a [TurtleBot3 Burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview) can be launched wherefore your system requires the following two meta-packages: [turtlebot3](http://wiki.ros.org/turtlebot3) and [turtlebot3_simulations](http://wiki.ros.org/turtlebot3_simulations). Also, an "ideal" robot with is added which is just a hovering box that has no direct ground contact and utilizes [Gazebo's planar move plugin](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins\#PlanarMovePlugin) which removes any problems originating from the simulated skid-steering of the Husky robot.

The simulated robots have a 2D laser scanner used for SLAM with [GMapping](http://wiki.ros.org/slam_gmapping) (must be installed as well) and a simulated Intel RealSense depth camera for building the required OctoMap. A second Husky configuration features a [simulated Velodyne VLP-16 lidar](http://wiki.ros.org/velodyne_simulator). To run the simulations, follow the steps below.

Furthermore, you need [OctoMap](http://wiki.ros.org/octomap) for gain evaluation and the RSM melodic-devel branch to run the exploration.

If you did not already have the below installed, do so now:

```
# Husky simulation with Velodyne
sudo apt-get install ros-melodic-husky-simulator ros-melodic-velodyne-simulator ros-melodic-velodyne

#Turtlebot3 simulation
sudo apt-get install ros-melodic-turtlebot3 ros-melodic-turtlebot3-simulations

#GMapping for SLAM, Navigation, OctoMap and Hector Trajectory Server for visualization in RViz
sudo apt-get install ros-melodic-slam-gmapping ros-melodic-navigation ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-rviz-plugins ros-melodic-hector-trajectory-server

cd /path/to/your/catkin_ws/src
git clone https://github.com/MarcoStb1993/robot_statemachine.git --branch melodic-devel
catkin build robot_statemachine
```

Now you can launch RNE using one of the following commands (the first for the Husky with a depth camera, the second for the Husky with a lidar, the third for the Turtlebot3, the fourth for the "ideal" robot with a depth camera and the fifth for the "ideal" robot with a lidar):

```
roslaunch rsb_nbv_exploration_plugins simulation_husky_realsense.launch

roslaunch rsb_nbv_exploration_plugins simulation_husky_velodyne.launch

roslaunch rsb_nbv_exploration_plugins simulation_turtlebot3.launch 

roslaunch rsb_nbv_exploration_plugins simulation_realsense.launch

roslaunch rsb_nbv_exploration_plugins simulation_velodyne.launch
```

Start and stop the exploration using the RSM GUI plugin for Rviz which is automatically shown.

You can change the `min_view_score` using dynamic reconfigure (start it with `rosrun rqt_reconfigure rqt_reconfigure`). You can load another Gazebo world by supplying its path to the `world` parameter.

**Note**: Be aware that the simulated depth camera and Velodyne only clear free space in the OctoMap for a ray if the ray ends at an obstacle. This requires comparingly large values for `min_view_score` in open spaces (f.e. 0.5 for the Husky simulation above). Values between 0.05 and 0.15 were found to lead to good results in confined spaces.

## AEDE interface

AEDE can be used as an alternative to the ROS navigation stack. Therefore, AEDE's local planner and terrain analysis packages have to be installed using the commands below:

```
de /path/to/your/catkin_ws/src
git clone https://github.com/HongbiaoZ/autonomous_exploration_development_environment --branch melodic
catkin build local_planner terrain_analysis
```

A simulation using the AEDE local planner can be started with the below commands (first for the "ideal" robot and second for a Husky robot):

```
roslaunch rsb_nbv_exploration_plugins simulation_aede.launch

roslaunch rsb_nbv_exploration_plugins simulation_aede_husky.launch
```
Afterwards, in another terminal, run the following command to start the exploration. Since the AEDE interface is not using RSM, no GUI plugin for RViz is available:

```
rosservice call /rne/setRneState "data: true"
```


### AedeInterfaceNode
This interface is used for local planning including terrain analysis. It requests the RNE's path to the current goal and uses the AEDE local planner to swiftly follow it.

#### Published Topics

**way_point** ([geometry_msgs/PointStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PointStamped.html))  
The position to which to plan to for the AEDE local planner

**stop** ([std_msgs/Int8](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int8.html))  
If a number above 0 is published, AEDE's path follower stops moving the robot

**path** ([nav_msgs/Path](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Path.html))  
The path towards the goal position

#### Subscribed Topics

**explorationGoalObsolete** ([rsb_nbv_exploration_msgs/ExplorationGoalObsolete](../rsb_nbv_exploration_msgs/msg/ExplorationGoalObsolete.msg))  
If the current goal is still the best goal and if it was already requested by navigation

#### Parameters

**~update_frequency** (float, default: 1)  
Update rate in Hz

**~robot_frame** (string, default: "base_footprint")  
Frame name of the robot's base frame

**~position_tolerance** (float, default: 0.1)  
Tolerance in m to count as being at a way point which is twice that of a goal to enable faster switching to the next way point

**~idle_timer_duration** (float, default: 0.5)  
Time in s that the robot can remain stationary before navigation counts as aborted because the robot is stuck
 
### AedeMessageInterface
 
This message interface refactors messages for AEDE. The required point cloud for terrain analysis is transformed from the sensor to the map frame and the command velocity is changed from a TwistStamped to a Twist message.

#### Published Topics

**cmd_vel_stamped** ([geometry_msgs/TwistStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistStamped.html))  
The commanded velocity for the robot with a message header

**registered_velodyne_points** ([sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html))  
Velodyne lidar point cloud in map frame

#### Subscribed Topics

**cmd_vel** ([geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))  
The commanded velocity for the robot

**velodyne_points** ([sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html))  
Velodyne lidar point cloud in sensor frame

#### Parameters

**~robot_frame** (string, default: "base_footprint")  
Frame name of the robot's base frame

#### Required tf Transforms

**map -> <velodyne_frame>**  
Usually provided by SLAM
