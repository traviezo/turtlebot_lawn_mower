# turtlebot_lawn_mower

[![Build Status](https://travis-ci.org/Traviezo/turtlebot_lawn_mower.svg?branch=master)](https://travis-ci.org/Traviezo/turtlebot_lawn_mower)
[![Coverage Status](https://coveralls.io/repos/github/traviezo/turtlebot_lawn_mower/badge.svg?branch=master)](https://coveralls.io/github/traviezo/turtlebot_lawn_mower?branch=master)
---

## Overview

The Turtlebot Lawn Mower project is based on a gazebo simulation of the turtlebot as a robotic lawn mower. The robotic simulation system is capable of mapping the desired lawn area to be cut, follow a predefined pattern to cut the lawn and avoid obstacles in its surroundings while completing its task. At the end of its task, it will return to its hub where it originally started from and be ready for next use.

## Detailed Description

The Turtlebot lawn mower simulation project is designed using the simultaneous localization and mapping ros algorithms (SLAM). It makes use of the Actionlib, gmapping and amcl packages. The task of the Turtlebot lawn mower will be to first map the area to work on. Then it should localized itself within this map with the aid of simulated laser sensors and amcl algorithms in the gazebo simulation. The Turtlebot then will move to the start position of the mowing pattern and start moving along the pattern simulating that is cutting grass as it moves. When task is completed, the lawn mower should returned to its original hub or starting position which is depicted in the gazebo simulation at the right back corner of the house yard.

System UML Diagrams and activity diagrams are stored in the repository in order to provide a more clear explanation of how the simulation and nodes work.

## Steps to Run Simulation
 In order to run the simulation follow steps below:
1. Clone project from repository:
   $ git clone www.github.com/traviezo/turtlebot_lawn_mower
2. Place project under catkin_ws/src folder
3. Build project:
   $ catkin_make
4. Run roslaunch file containing other roslaunch files that setup the gazebo world, rviz and amcl node.
   $ roslaunch turtlebot_lawn_mower turtlebot_lawn_mower.launch 
5. Run the turtlebot_lawn_mower node in a separate window terminal:
   $ rosrun turtlebot_lawn_mower turtlebot_lawn_mower_node.
6. Turtlebot should take about 6-7 minutes to complete its task.
7. When turtlebothas return to its hub, you can terminate the simulation by pressing Ctrl+c in the terminal window. 

## Steps to Run ROS test units
 The following should be taken in order to run the ros tests for this simulation:
1. Clone project from repository:
   $ git clone www.github.com/traviezo/turtlebot_lawn_mower
2. Place project under catkin_ws/src folder
3. Build project:
   $ catkin_make tests
4. Run the lunch file containing the rostest node:
   $ roslaunch turtlebot_lawn_mower turtlebotTest.launch 
5. After tests are run and passed you can stop the node pressing Ctrl+c

## Backlog Google Spreadsheet Link

https://docs.google.com/spreadsheets/d/1ZgBcRyQ3cb3LdhyaTrL4cho9RW_72Gf_x5xqKgn96SY/edit?usp=sharing

## Power Point Presentation containing video demo Link
https://docs.google.com/presentation/d/14n2nd_QIcrkeqKLVICN0gg27taTeCZKI521rLPs3wX8/edit?usp=sharing





