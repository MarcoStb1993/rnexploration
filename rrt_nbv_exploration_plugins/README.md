# RRT NBV Exploration Plugins

This package bundles plugins for the [RRT NBV Exploration package](../rrt_nbv_exploration#rrt-nbv-exploration) that can be used to run the package together with the [Robot Statemachine (RSM) package](http://wiki.ros.org/robot_statemachine) and the [ROS navigation stack](http://wiki.ros.org/navigation).

Furthermore, it includes convenient configuration, URDF and launch-files to start a simulation with RNE straight away.

## RSM plugin

To interface the RSM, a plugin state called RnExplorationState was implemented following the [RSM's tutorial for writing a plugin state](http://wiki.ros.org/robot_statemachine/Tutorials/WritingAPluginState). This plugin state requests the current goal from the TreeConstructor class and sets it as the next navigation goal, then triggers a transition to the navigation state.

Furthermore, a node to integrate RNE into the RSM framework was written which is called RneServiceProviderNode. Basically, it receives the status of ROS navigation and forwards to RNE if a goal was reached or reaching it failed. If the exploration mode is set to *interrupt* it also checks if a goal became obsolete and tells the navigation to abort it.

### RneServiceProviderNode

This node handles the data transition between RSM and RNE because the plugin states are volatile and not able to store data. 

#### Published Topics

**goalObsolete** ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))  
Information if the current goal is still viable (only active if exploration mode is set to *interrupt*)

#### Subscribed Topics

**bestAndCurrentGoal** ([rrt_nbv_exploration_msgs/BestAndCurrentNode](../rrt_nbv_exploration_msgs/msg/BestAndCurrentNode.msg))  
Current exploration goal and node with best gain-cost-ratio

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

A global planner which uses the edges between the robot and the goal node along the RRT as a path was implemented following the [tutorial provided by navigation](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS). It retrieves the path from the TreeConstructor class if exploration is running, if not, the standard global planner is used.

## Starting RNE

This package includes a configuration and URDF to launch a gazebo based simulation of RNE deploying a [Clearpath Robotics Husky](http://wiki.ros.org/Robots/Husky) robot which simulation is available as a [ROS package](http://wiki.ros.org/husky_simulator?distro=melodic) and is required to run this launch file.

The simulated robot has a 2D laser scanner used for SLAM with [GMapping](http://wiki.ros.org/slam_gmapping) (must be installed as well) and a simulated Intel RealSense depth camera for building the required OctoMap. To run the simulation, follow the steps below.

Furthermore, you need [OctoMap](http://wiki.ros.org/octomap) for gain evaluation and the RSM melodic branch to run the exploration.

If you did not already have the below installed, do so now:

```
sudo apt install ros-melodic-husky-simulator
sudo apt install ros-melodic-slam-gmapping
sudo apt install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-rviz-plugins

cd /path/to/your/catkin_ws/src
clone https://github.com/MarcoStb1993/robot_statemachine.git --branch melodic-devel
catkin build robot_statemachine
```

Now you can launch RNE using the following command:

```
roslaunch rrt_nbv_exploration_plugins simulation_realsense.launch rviz:=true edge_length:=1.0
```

If `rviz` is set to true, the RViz configuration including the RSM RViz plugin and all required displays is loaded. You can change the `min_view_score` using dynamic reconfigure (start it with `rosrun rqt_reconfigure rqt_reconfigure`). You can load another gazebo world by supplying its path to the `world` parameter.



