cmake_minimum_required(VERSION 2.8.3)
project(rrt_nbv_exploration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    pluginlib
  	rsm_core
  	rsm_msgs
  	std_msgs
  	std_srvs
    move_base_msgs
    roscpp
    rospy
    geometry_msgs
    visualization_msgs
    rrt_nbv_exploration_msgs
    tf2
    tf2_ros
    nav_msgs
    dynamic_reconfigure
    )

find_package(octomap REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/GainCalculator.cfg
)

catkin_package(
	INCLUDE_DIRS include
    )

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    )

link_libraries(${OCTOMAP_LIBRARIES})

add_executable(rneNode src/RneNode.cpp
				src/TreeConstructor.cpp
				src/TreeSearcher.cpp
				src/CollisionChecker.cpp
				src/TreePathCalculator.cpp
				src/NodeComparator.cpp)
add_executable(gainCalcNode src/GainCalcNode.cpp
				src/GainCalculator.cpp)
add_executable(rneVisualizationNode src/RneVisualizationNode.cpp
				src/RneVisualizer.cpp)


target_link_libraries(rneNode
    ${catkin_LIBRARIES}
    )
target_link_libraries(gainCalcNode
    ${catkin_LIBRARIES}
    )
target_link_libraries(rneVisualizationNode
    ${catkin_LIBRARIES}
    )

add_dependencies(rneNode ${catkin_EXPORTED_TARGETS})
add_dependencies(gainCalcNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(rneVisualizationNode ${catkin_EXPORTED_TARGETS})
