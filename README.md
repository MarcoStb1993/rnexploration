# Random-Sampling-Based Next-Best View Exploration (RNE)

This package contains an exploration algorithm based on the Rapidly-Exploring Random Graph (RRG) to randomly sample viewpoints the configuration space. 2D grid maps or 2.5D traversability grid maps can be supplied to RNE to assess if a robot can reach new nodes in the graph and to move nodes away from obstacles to create a topology-based graph. Sparse Ray Sampling (SRP) in an OctoMap is used to evaluate each node's gain. Sampling and evaluation are executed in a sliding area around the robot.

Nodes that fall outside of this area are removed from the local graph. If they are still worth exploring, they are added to the global graph as a target. These targets are clustered and connected to each other and the local graph using removed local nodes as waypoints. When no nodes worth exploring are left in the local area, a Traveling Salesman Problem (TSP) solver is deployed to determine the first global target to explore. Upon reaching it, the local exploration is re-initialized at the target's position.

Below two screenshots from gazebo and RViz are shown with RNE running in a simulation of an underground cave environment.

![Simulated underground cave environment](images/cave_gazebo.png)
![RNE and OctoMap visualization for the shown simulation](images/cave_rviz.png)

The RNE contains a plugin to use it as an exploration algorithm in the [Robot Statemachine (RSM) package](http://wiki.ros.org/robot_statemachine) and a global planner plugin for the [ROS navigation stack](http://wiki.ros.org/navigation).

Details about the RNE's implementation can be found in the [RSG NBV Exploration package](rsb_nbv_exploration#rsb-nbv-exploration) and about the plugins in the [RSG NBV Exploration Plugins package](rsb_nbv_exploration_plugins#rsb-nbv-exploration-plugins). The messages and services defined for RNE are in the [RSG NBV Exploration Msgs package](rsb_nbv_exploration_msgs#rsb-nbv-exploration-messages). The plugins package also contains convenient config and launch files to run a simulation with RNE in gazebo.

## Citing

If you use RNE in an academic context, please cite the following [publication](https://ieeexplore.ieee.org/document/9568785):

```
@INPROCEEDINGS{RNE,
author={Steinbrink, Marco and Koch, Philipp and Jung, Bernhard and May, Stefan},
booktitle={2021 European Conference on Mobile Robots (ECMR)},   
title={Rapidly-Exploring Random Graph Next-Best View Exploration for Ground Vehicles},   
year={2021},  
volume={},  
number={},  
pages={1-7},  
doi={10.1109/ECMR50962.2021.9568785}}
```

Pre-print available on [arXiv](https://arxiv.org/abs/2108.01012).
