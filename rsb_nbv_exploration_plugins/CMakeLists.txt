cmake_minimum_required(VERSION 2.8.3)
project(rsb_nbv_exploration_plugins)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    pluginlib
  	rsm_core
  	rsm_msgs
  	std_msgs
  	std_srvs
  	geometry_msgs
  	rsb_nbv_exploration_msgs
  	tf2_geometry_msgs
  	tf2
    tf2_ros
  	costmap_2d
  	nav_core
  	base_local_planner
  	angles
  	nav_msgs
  	global_planner
  	sensor_msgs
)

catkin_package(
	INCLUDE_DIRS include
    )

include_directories(include
    ${catkin_INCLUDE_DIRS}
    )
		
add_library(rne_plugins src/RnExplorationState.cpp)	

add_library(rne_global_planner src/RneGlobalPlanner.cpp)

add_executable(rneServiceProviderNode
	src/RneServiceProviderNode.cpp
	src/RneServiceProvider.cpp)	
	
add_executable(aedeInterfaceNode
	src/AedeInterfaceNode.cpp
	src/AedeInterface.cpp)	
	
	
add_executable(aedeMessageInterfaceNode
	src/AedeMessageInterfaceNode.cpp
	src/AedeMessageInterface.cpp)	
	
target_link_libraries(rne_plugins
  ${catkin_LIBRARIES}
  )
  
target_link_libraries(rne_global_planner
  ${catkin_LIBRARIES}
  )
  
target_link_libraries(rneServiceProviderNode
  ${catkin_LIBRARIES}
  )
  
target_link_libraries(aedeInterfaceNode
  ${catkin_LIBRARIES}
  )
  
target_link_libraries(aedeMessageInterfaceNode
  ${catkin_LIBRARIES}
  )
  
add_dependencies(rne_plugins ${catkin_EXPORTED_TARGETS})
add_dependencies(rne_global_planner ${catkin_EXPORTED_TARGETS})
add_dependencies(rneServiceProviderNode ${catkin_EXPORTED_TARGETS})
add_dependencies(aedeInterfaceNode ${catkin_EXPORTED_TARGETS})
add_dependencies(aedeMessageInterfaceNode ${catkin_EXPORTED_TARGETS})
