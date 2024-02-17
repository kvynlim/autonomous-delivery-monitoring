# Autonomous Delivery Monitoring
This project is split into 3 sub-folders
* api-server
* lib
* ros_ws

# Requirements

* Ubuntu 20.04
* ROS Noetic
* Python 3.8.10

# The Python Package Index (PyPI) installations
```
pip3 install -r requirements.txt
```

# Steps to run project

1. Build the ros packages by running catkin_make in ros_ws directory

> Ensure to source devel/setup.bash before proceeding

2. Execute the following command to launch gazebo simulation
```
roslaunch turtlebot3_gazebo multi_turtlebot3.launch
```
3. Execute main.py using python3 in api-server directory to start the API server
* Visit http://0.0.0.0:5000/docs to test out the API

4. Execute test.py using python3 in api-server/test directory to run the test script