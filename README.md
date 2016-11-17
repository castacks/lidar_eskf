# README #

This README should able to help you setup and run this program.

### What is this repository for? ###

**lidar_eskf** is a package for localization or pose tracking with g2 and microstrain. The localization is cast into an ESKF framework, while a GPF is used to encode range measurements. 

### How do I get set up? ###

Install ros octomap: 

```
sudo apt-get install ros-indigo-octomap*
```

Create workspace ```test_ws```, fetch repositories and build: 

```bash 
sudo apt-get install python-wstool # get wstool
source /opt/ros/indigo/setup.bash # init environment
mkdir -p test_ws/src
cd test_ws/src
catkin_init_workspace
git clone git@bitbucket.org:castacks/metarepository.git
ln -s metarepository/rosinstall/inspect_dji.rosinstall .rosinstall
wstool info # see status
wstool up # get stuff
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```
### Executables info? ###
**bag_to_pcd**: Stacking laser topics from a ros bagfile into pcd file. See the ```bag_pcd_saver.launch``` for more information.

**lidar_eskf_node**: The main estimation program.

### How do I run? ###


### Who do I talk to? ###
Weikun Zhen (zhenwk@gmail.com)