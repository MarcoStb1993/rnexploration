# RRT NBV Exploration

## How to install FCL


Install Flexible Collision Library (FCL) for ROS Kinetic from [github](https://github.com/flexible-collision-library/fcl/tree/master) with the following commands in a directory of your choice (f.e. home). It requires the Eigen (should be installed already) library to be installed, libccd (install with `sudo apt-get -qq install libccd-dev`) and octomap ([ROS package](http://wiki.ros.org/octomap) and `sudo apt install liboctomap-dev`):

```
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
git checkout fcl-0.5
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

Now add the following lines to your *CMakeLists.txt*:

```
find_package(fcl REQUIRED)

include_directories(include
    ...
    ${fcl_INCLUDE_DIRS}
    )
    
link_libraries(... ${fcl_LIBRARY_DIRS})
    
 target_link_libraries(yourExecutableOrLibrary
    ... ${fcl_LIBRARIES} fcl
    )
```
