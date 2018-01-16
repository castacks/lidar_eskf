# README #

### What is this repository for? ###

**lidar_eskf** is a package for localization or pose tracking with g2 (rotating 2d lidar) and microstrain (imu). The localization is cast into an ESKF framework, while a GPF is used to encode range measurements. 

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

### License ###
[This software is BSD licensed.](http://opensource.org/licenses/BSD-3-Clause)

Copyright (c) 2017, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.