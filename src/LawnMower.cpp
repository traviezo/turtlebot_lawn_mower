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
 * @file LawnMower.cpp
 * @author Christian Ramos
 * @date 8 May 2017
 * @brief File implements the class LawnMower
 *
 * This file implements all member functions declared in
 * LawnMower header file.  
 * 
 * @see https://github.com/traviezo/turtlebot_lawn_mower
 */

#include "../include/LawnMower.h"
#include <vector>

void LawnMower::setPathX(std::vector<double> x)
{
  pathXCoordinates = x;
}

void LawnMower::setPathY(std::vector<double> y)
{
  pathYCoordinates = y;
}

void LawnMower::setAnglePose(std::vector<double> theta)
{
  pathAnglePose = theta;
}

void LawnMower::setPoseGoals(double xPose, double yPose, geometry_msgs::Quaternion qMsg, move_base_msgs::MoveBaseGoal & goal)
{
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = xPose;
  goal.target_pose.pose.position.y = yPose;
  goal.target_pose.pose.orientation = qMsg;
}

void LawnMower::sendGoal(move_base_msgs::MoveBaseGoal goal, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &actionClient)
{
  ROS_INFO("Sending robot to:x= %f, y= %f, theta= %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation);
  actionClient.sendGoal(goal);
}

tf::Quaternion LawnMower::quaternionCalculation(double theta)
{
  double radians = theta*(M_PI/180);
  tf::Quaternion quaternion;
  quaternion = tf::createQuaternionFromYaw(radians);
  return quaternion;
}

geometry_msgs::Quaternion LawnMower::convertToQMsg(tf::Quaternion quaternion)
{
  geometry_msgs::Quaternion qMsg;
  tf::quaternionTFToMsg(quaternion, qMsg);
  return qMsg;
}


