cmake_minimum_required(VERSION 2.8.3)
project(rrt_nbv_exploration_plugins)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
)

catkin_package(CATKIN_DEPENDS pluginlib roscpp
               INCLUDE_DIRS include
               LIBRARIES plugin_steering_husky)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(plugin_steering_husky src/plugin_steering_husky)
