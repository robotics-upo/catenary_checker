# catenary_checker
Simple algorithm for check whether a catenary can be found in a environment in such way that it does not collide with obstacles

## Installation and dependencies

The code presented here has been tested in Ubuntu 20.04, with ROS Noetic and requires qt5-charts and qt5-widgets libraries.

To compile it, please download it into the src folder of your favourite catkin_ws. 

```
  cd ~/catkin_ws/src
  git clone https://github.com/robotics-upo/catenary_checker
```

Then, use a compilation utility to build the binaries (for example catkin_make).

```
cd ~/catkin_ws
catkin_make
```

This code depends on our DBScan-Line clustering algorithm, which is available at:

https://github.com/robotics-upo/dbscan_line

Please follow the instructions there to install it into your system

### A simple test

To run a simple test, make sure that the ROS environment variables are loaded (source the setup.bash file of your catkin workspace) and then simply execute:

```
rosrun catenary_checker test_catenary_checker
```