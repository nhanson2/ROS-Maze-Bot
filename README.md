# ROS-Maze-Bot

![Sample Maze](./project/img/maze.png)

A Robot that solves mazes. What's not to love?

## Installation

System Specifications
* Ubuntu 20.0.4 Focal Fossa
* ROS Neotic (Latest)

```
source /opt/ros/noetic/setup.bash 
cd ~/ROS-Maze-Bot/catkin_ws
catkin_make
export TURTLEBOT3_MODEL=burger
### Launch Simulation Environment
roslaunch fira_maze maze_1_world.launch
### Launch Robot Controller
rosrun fira_maze maze_explorer.py 
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
