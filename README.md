# ROS-Maze-Bot

![Sample Maze](./project/img/maze.png)

A Robot that solves mazes. What's not to love?

## Installation

System Specifications
* Ubuntu 18.04.5 LTS. Release: 18.04 "bionic"
* ROS Melodic

Package Dependencies
* slam_gmapping     
`$sudo apt install ros-melodic-slam-gmapping`
* map_server       
`$ sudo apt install ros-melodic-map-server`
* amcl              
`$ sudo apt install ros-melodic-amcl`
* navigation       
`$ sudo apt-get install ros-melodic-navigation`

```
source /opt/ros/melodic/setup.bash 
cd ~/ROS-Maze-Bot/catkin_ws
catkin_make
export TURTLEBOT3_MODEL=burger
### Part 1
roslaunch fira_maze part1.launch

### Part 2
roslaunch fira_maze part2.launch

### Part 3
roslaunch fira_maze part3.launch

### Creativity Portion (Multi-Robot Mapping)
roslaunch fira_maze creativity.launch

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
Final editing was done in Overleaf. The final report is completed as a .pdf and can be found in the root directory.

## Presentation
Our video presentation can be found at the following link: https://youtu.be/OeK8udU62_8

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

#### 12-02-2020
- Completed part 1

#### 12-07-2020
- Completed part 2

#### 12-17-2020
- Completed creativity step (part 4)
