cmake_minimum_required(VERSION 2.8.3)
project(rrt_nbv_exploration_plugins)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    pluginlib
  	rsm_core
  	rsm_msgs
  	geometry_msgs
  	rrt_nbv_exploration_msgs
  	tf
)

catkin_package(
	INCLUDE_DIRS include
    )

include_directories(include
    ${catkin_INCLUDE_DIRS}
    )
		
add_library(rne_plugins src/RnExplorationState.cpp)		
	
target_link_libraries(rne_plugins
  ${catkin_LIBRARIES}
  )
  
add_dependencies(rne_plugins ${catkin_EXPORTED_TARGETS})
