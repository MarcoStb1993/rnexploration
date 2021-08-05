cmake_minimum_required(VERSION 2.8.3)
project(rrg_nbv_exploration_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    visualization_msgs
    geometry_msgs
    message_generation
    )

 add_message_files(
   FILES
   Graph.msg
   Node.msg
   Edge.msg
   SphericalPoint.msg
   Cluster.msg
   BestAndCurrentNode.msg
   NodeToUpdate.msg
   )
   
add_service_files(
  FILES  
  RequestGoal.srv
  RequestPath.srv
  UpdateCurrentGoal.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
