/****************************************************************************** 
 * Copyright (C) 2017 by Christian Ramos				      *
 * The MIT License							      *
 * Permission is hereby granted, free of charge, to any person obtaining a    *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction, including without limitation  *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,   *
 * and/or sell copies of the Software, and to permit persons to whom the      *
 * Software is furnished to do so, subject to the following conditions:       *
 *									      *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software. 			      *
 * 									      *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    *
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        *
 * DEALINGS IN THE SOFTWARE.						      *
 ******************************************************************************/

/**
 * @file turtlebot_lawn_mower.cpp
 * @author Christian Ramos
 * @date 5 May 2017
 * @brief File containing the main function
 *
 * This file calls the LawnMower class and other classes in order to
 * guide the turtlebot along the simulated gazebo world.  
 * 
 * @see https://github.com/traviezo/turtlebot_lawn_mower
 */

#include <vector>
#include "../include/LawnMower.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<double> xCoordinates { 0.25, 0.25,  2.75,  2.75, -1.25, -1.25,  3.25, 3.25, -1.25, -1.25,  3.25, 3.25, -1.25, -1.25,  3.25, 3.25, -1.25, -1.25,  3.25,  3.25, -0.5, -1.25, 3.25,  3.25};
std::vector<double> yCoordinates { 4.25, 3.75,  3.75,  3.25,  3.25,  2.75,  2.75, 2.25,  2.25,  1.75,  1.75, 1.25,  1.25,  0.75,  0.75, 0.25,  0.25, -0.25, -0.25,  -0.5, -1.5, -0.25, 4.25,  4.25};
std::vector<double> anglePose    {-90.0,  0.0, -90.0, 180.0, -90.0,   0.0, -90.0,  180, -90.0,   0.0, -90.0,  180, -90.0,   0.0, -90.0,  180, -90.0,   0.0, -90.0, 180.0, 90.0,   0.0,  0.0, 180.0};

int const NUMBER_OF_GOAL_POSES = 24;

int main(int argc, char** argv) {
  ros::init(argc, argv, "turtlebot_lawn_mower_node");
  ros::NodeHandle nh;
  LawnMower lawnMowerBot;
  move_base_msgs::MoveBaseGoal goal;
  MoveBaseClient actionClient("move_base", true);
  ROS_INFO("Waiting for the move_base action server...");
  actionClient.waitForServer(ros::Duration(60));
  ROS_INFO("Connected to move_base server");
  lawnMowerBot.setPathX(xCoordinates);
  lawnMowerBot.setPathY(yCoordinates);
  lawnMowerBot.setAnglePose(anglePose);
  for (int i = 0; i < NUMBER_OF_GOAL_POSES; i++) {
    tf::Quaternion quaternion;
    quaternion = lawnMowerBot.quaternionCalculation(lawnMowerBot.getPathAnglePose()->at(i));
    geometry_msgs::Quaternion qMsg;
    qMsg = lawnMowerBot.convertToQMsg(quaternion);
    lawnMowerBot.setPoseGoals(lawnMowerBot.getPathXCoordinate()->at(i), lawnMowerBot.getPathYCoordinate()->at(i), qMsg, goal);
    lawnMowerBot.sendGoal(goal, actionClient);
    actionClient.waitForResult();
    if (actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("You have reached goal..");
    else
      ROS_INFO("The base failed to reach goal..");
  }
  return 0;
}
