# turtlebot_lawn_mower

[![Build Status](https://travis-ci.org/Traviezo/turtlebot_lawn_mower.svg?branch=master)](https://travis-ci.org/Traviezo/turtlebot_lawn_mower)
[![Coverage Status](https://coveralls.io/repos/github/traviezo/turtlebot_lawn_mower/badge.svg?branch=master)](https://coveralls.io/github/traviezo/turtlebot_lawn_mower?branch=master)
---

## Overview

The Turtlebot Lawn Mower project is based on a gazebo simulation of the turtlebot as a robotic lawn mower. 

## Detailed Description

The Turtlebot lawn mower simulation project will be designed using the simultaneous localization and mapping ros algorithms (SLAM). The task of the Turtlebot lawn mower will be to first map the area to work on. Then it should localized itself within this map with the aid of simulated laser sensors provided by the gazebo simulation. The Turtlebot then will move to the start position of the mowing pattern and start moving along the pattern simulating that is cutting grass as it moves. When task is completed, the lawn mower should publish a string message to the ROS network to acknoledge that it has accomplished its mission.



## Backlog Google Spreadsheet Link

https://docs.google.com/spreadsheets/d/1ZgBcRyQ3cb3LdhyaTrL4cho9RW_72Gf_x5xqKgn96SY/edit?usp=sharing


