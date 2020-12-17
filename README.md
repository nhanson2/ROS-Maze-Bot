# ROS-Maze-Bot

![Sample Maze](./project/img/maze.png)

A Robot that solves mazes. What's not to love?

## Installation

System Specifications
* Ubuntu 18.04.5 LTS. Release: 18.04 "bionic"
* ROS Melodic (Latest)

Package Dependencies
* slam_gmapping     $ sudo apt install ros-melodic-slam-gmapping
* map_server        $ sudo apt install ros-melodic-map-server
* amcl              $ sudo apt install ros-melodic-amcl
* navigation        $ sudo apt-get install ros-melodic-navigation

```
source /opt/ros/melodic/setup.bash 
cd ~/ROS-Maze-Bot/catkin_ws
catkin_make
export TURTLEBOT3_MODEL=burger
### Launch Simulation Environment/SLAM/Movement Script
roslaunch fira_maze maze_1_world.launch
### Launch rviz to visualize mapping
rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_gmapping.rviz

### After mapping is complete, save the map
# replace <map_filename> with the filename you'd like to save for the map
rosrun map_server map_saver -f /home/ubuntu/ROS-Maze-Bot/maps/<map_filename>
# This outputs a .pgm and a .yaml
```

## Workflow

This repository has two branches *develop* and *master*. All development should take place on *develop*. As the active development location, it represets a work in progress. Work will be migrated to *Master* when it has been tested and confirmed to be stable. 

The specifc sequence of commands should be followed when working on code in the repository:

```
# First time only
git clone <<REPO ADDRESS>>

# Before you start working - update your local repository with changes from GitHub
git pull

# After you are done making changes
git add  <<FILES YOU CREATED/CHANGED>>
git commit -m "<<MESSAGE EXPLAINING WHAT CHANGES YOU MADE>>
git push
```

## Reporting

Compiled using TeXworks Version 0.6.5 (MiKTeX 20.7) and following the ICRA formatting guidelines found at the following link: http://ras.papercept.net/conferences/support/tex.php
root.tex is the report TeX file. The generated pdf document can be viewed with any appropriate pdf viewer.

## Status Updates

#### 11-13-2020
- Initial Project Setup
- Further Configured Git Settings
- Created additional users on Ubuntu server
#### 11-16-2020
- Updated README.md to include information on package dependencies, and instructions on utilizing slam, rviz, and mapping.
- Added sample map files created in simulation to maps folder.
#### 11-19-2020
- Included AMCL launch file (work in progress).
- Changed maze_1_world.launch file to incoporate SLAM and movement script.
- Updated README.md to reflect changes.
