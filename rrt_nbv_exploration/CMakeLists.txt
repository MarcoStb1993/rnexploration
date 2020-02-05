cmake_minimum_required(VERSION 2.8.3)
project(rrt_nbv_exploration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    pluginlib
    move_base_msgs
    roscpp
    std_msgs
    geometry_msgs
    visualization_msgs
    rrt_nbv_exploration_msgs
    )

find_package(octomap REQUIRED)

catkin_package(
	INCLUDE_DIRS include
    )

include_directories(include
    ${catkin_INCLUDE_DIRS}
    )

link_libraries(${OCTOMAP_LIBRARIES})

add_executable(rneNode src/RneNode.cpp
				src/TreeConstructor.cpp
				src/TreeSearcher.cpp
				src/GainCalculator.cpp
				src/Navigator.cpp
				src/CollisionChecker.cpp)
add_executable(rneVisualizationNode src/RneVisualizationNode.cpp
				src/RneVisualizer.cpp)

target_link_libraries(rneNode
    ${catkin_LIBRARIES}
    )
target_link_libraries(rneVisualizationNode
    ${catkin_LIBRARIES}
    )

add_dependencies(rneNode ${catkin_EXPORTED_TARGETS})
add_dependencies(rneVisualizationNode ${catkin_EXPORTED_TARGETS})
