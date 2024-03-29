cmake_minimum_required(VERSION 2.8.3)
project(rsb_nbv_exploration)

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
    rsb_nbv_exploration_msgs
    tf2
    tf2_ros
    nav_msgs
    map_msgs
    dynamic_reconfigure
    )

find_package(octomap REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/GainCalculator.cfg
  cfg/GraphConstructor.cfg
  cfg/RneVisualizer.cfg
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
				src/GraphConstructor.cpp
				src/GraphSearcher.cpp
				src/CollisionChecker.cpp
				src/GraphPathCalculator.cpp
				src/NodeComparator.cpp
				src/GlobalGraphHandler.cpp
				src/GlobalGraphSearcher.cpp
				src/GlobalGraphWaypointSearcher.cpp)
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

add_dependencies(rneNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(gainCalcNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(rneVisualizationNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
