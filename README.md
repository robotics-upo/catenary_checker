# catenary_checker
Simple algorithm for check whether a catenary can be found in a environment in such way that it does not collide with obstacles

## Installation and dependencies

The code presented here has been tested in Ubuntu 20.04, with ROS Noetic and requires qt5-charts and qt5-widgets libraries. To install them you can use apt-get (tested in Ubuntu 20.04):


```
  sudo apt-get install libqt5charts5-dev libqt5widgets5 
```

Moreover, it requires our wonderful DBScan-line library :
```
  git clone https://github.com/robotics-upo/dbscan_line
  cd dbscan_line
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
```


### Compile it with catkin:

To compile it, please clone the repository into the src folder of your favourite catkin_ws. For example:

```
  cd ~/catkin_ws/src
  git clone https://github.com/robotics-upo/catenary_checker
```

Then, use a compilation utility to build the binaries (for example catkin_make).

```
cd ~/catkin_ws
catkin_make
```

### A simple test

To run a simple test, make sure that the ROS environment variables are loaded (source the setup.bash file of your catkin workspace) and then simply execute:

```
rosrun catenary_checker test_catenary_checker
```
