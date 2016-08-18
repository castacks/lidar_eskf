# README #

This is a new laser IMU based localization package. It is different from ```lidar_ekf``` in that it's using error states to represent and propagate the system.

### How do I get set up? ###

Install ros octomap: 

```
sudo apt-get install ros-indigo-octomap ros-indigo-octomap-mapping
```

Launch:
```
roslaunch lidar_eskf lidar_eskf_node.launch
```
### Who do I talk to? ###
Weikun Zhen (zhenwk@gmail.com)