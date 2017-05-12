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
 * @file LawnMower.h
 * @author Christian Ramos
 * @date 8 May 2017
 * @brief File containing the Class LawnMower
 *
 * The class LawnMower stores coordinates that the turtlebot robot
 * will follow in order to acomplished its lawn mowing task. This class
 * also creates goals based on the coordinates data and controls the robot
 * path along the backyard simulated field in the gazebo simulator. 
 * @see https://github.com/traviezo/turtlebot_lawn_mower
 */

#ifndef _HOME_CHRISTIAN_CATKIN_WS_SRC_TURTLEBOT_LAWN_MOWER_INCLUDE_LAWNMOWER_H_
#define _HOME_CHRISTIAN_CATKIN_WS_SRC_TURTLEBOT_LAWN_MOWER_INCLUDE_LAWNMOWER_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <vector>

class LawnMower {
 public:
/** @brief Stores the x coordinates of desired lawn mowing pattern
  * @param x data coordinates of x axis  
  * @return void function does not return anything
  */
    void setPathX(std::vector<double> x);

/** @brief Stores the y coordinates of desired lawn mowing pattern
  * @param y data coordinates of y axis  
  * @return void function does not return anything
  */
    void setPathY(std::vector<double> y);

/** @brief Stores the pose angle of orientation of desired lawn mowing pattern
  * @param theta pose angle of orientation 
  * @return void function does not return anything
  */
    void setAnglePose(std::vector<double> theta);

/** @brief Sets a goal coordinate in the x,y plane and pose angle
  * @param xPose goal x coordinate
  * @param yPose goal y coordinate
  * @param qMsg quaternion angle pose message 
  * @param goal stores goal coordinates and pose angle
  * @return void function does not return anything
  */
    void setPoseGoals(double xPose, double yPose, geometry_msgs::Quaternion qMsg, move_base_msgs::MoveBaseGoal & goal);

/** @brief Broadcasts a goal action to the ros network
  * @param goal desired location for turtlebot robot
  * @param actionClient Object that broadcasts goal to network 
  * @return void function does not return anything
  */
    void sendGoal(move_base_msgs::MoveBaseGoal goal, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &actionClient);

/** @brief Calculates conversion from Euler angle to Quaternion.
  * @param theta pose yaw angle. 
  * @return tf::Quaternion quaternion containing goal pose data.
  */
    tf::Quaternion quaternionCalculation(double theta);

/** @brief Converts quaternion to a more suitable message for broadcasting.
  * @param quaternion quaternion containing goal pose data.
  * @return geometry_msgs::Quaternion quaternion message that contains goal pose data.
  */
    geometry_msgs::Quaternion convertToQMsg(tf::Quaternion quaternion);

/** @brief returns specific specified element from x coordinate vector.
  * @param none 
  * @return const std::vector<double>* pointer to element of x coordinate vector.
  */
    const std::vector<double>* getPathXCoordinate() const {return &pathXCoordinates;}

/** @brief returns specific specified element from y coordinate vector.
  * @param none 
  * @return const std::vector<double>* pointer to element of y coordinate vector.
  */
    const std::vector<double>* getPathYCoordinate() const {return &pathYCoordinates;}

/** @brief returns specific specified element from pose angle vector.
  * @param none 
  * @return const std::vector<double>* pointer to element of pose angle vector.
  */
    const std::vector<double>* getPathAnglePose() const {return &pathAnglePose;}

 private:
    std::vector<double> pathXCoordinates;    ///< goal x coordinates vector
    std::vector<double> pathYCoordinates;    ///< goal y coordinates vector
    std::vector<double> pathAnglePose;       ///< goal pose angle vector
};
#endif  // _HOME_CHRISTIAN_CATKIN_WS_SRC_TURTLEBOT_LAWN_MOWER_INCLUDE_LAWNMOWER_H_
