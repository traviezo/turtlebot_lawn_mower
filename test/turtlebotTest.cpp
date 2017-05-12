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
 * @file turtlebotTest.cpp
 * @author Christian Ramos
 * @date 10 May 2017
 * @brief File contains tests for the LawnMower class
 *
 * This file tests the LawnMower class member functions.  
 * Member functions that do not have return parameters are not tested,
 * instead visual inspection is performed on this functions using
 * ROS_INFO messages in the source code in order to provide the user a
 * sense of parameters confidence while running the simulation.
 * 
 * @see https://github.com/traviezo/turtlebot_lawn_mower
 */

#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include "../src/LawnMower.cpp"

TEST(QuaternionConversion, CorrectQuaternionCalculation1)
{
  LawnMower lawnMowerBot;
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  EXPECT_EQ(q, lawnMowerBot.quaternionCalculation(0.0));
}

TEST(QuaternionConversion, CorrectQuaternionCalculation2)
{
  LawnMower lawnMowerBot;
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI);
  EXPECT_EQ(q, lawnMowerBot.quaternionCalculation(180.0));
}

TEST(QuaternionConversion, CorrectQuaternionCalculation3)
{
  LawnMower lawnMowerBot;
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, -(M_PI/2));
  EXPECT_EQ(q, lawnMowerBot.quaternionCalculation(-90.0));
}

TEST(QuaternionConversion, CorrectQuaternionCalculation4)
{
  LawnMower lawnMowerBot;
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, (M_PI/2));
  EXPECT_EQ(q, lawnMowerBot.quaternionCalculation(90.0));
}

TEST(PathCoordinates, getPathXCoordinate)
{
  LawnMower lawnMowerBot;
  std::vector<double> xCoordinates { 0.25, 0.25,  2.75,  2.75, -1.25, -1.25,  3.25, 3.25, -1.25, -1.25,  3.25, 3.25, -1.25, -1.25,  3.25, 3.25, -1.25, -1.25,  3.25,  3.25, -0.5, -1.25, 3.25,  3.25};
  lawnMowerBot.setPathX(xCoordinates);
  EXPECT_EQ(2.75, lawnMowerBot.getPathXCoordinate()->at(3));
}

TEST(PathCoordinates, getPathYCoordinate)
{
  LawnMower lawnMowerBot;
  std::vector<double> yCoordinates { 4.25, 3.75,  3.75,  3.25,  3.25,  2.75,  2.75, 2.25,  2.25,  1.75,  1.75, 1.25,  1.25,  0.75,  0.75, 0.25,  0.25, -0.25, -0.25,  -0.5, -1.5, -0.25, 4.25,  4.25};
  lawnMowerBot.setPathY(yCoordinates);
  EXPECT_EQ(3.25, lawnMowerBot.getPathYCoordinate()->at(3));
}

TEST(PathCoordinates, getPathAnglePose)
{
  LawnMower lawnMowerBot;
  std::vector<double> anglePose {-90.0,  0.0, -90.0, 180.0, -90.0,   0.0, -90.0,  180, -90.0,   0.0, -90.0,  180, -90.0,   0.0, -90.0,  180, -90.0,   0.0, -90.0, 180.0, 90.0,   0.0,  0.0, 180.0};
  lawnMowerBot.setAnglePose(anglePose);
  EXPECT_EQ(180, lawnMowerBot.getPathAnglePose()->at(3));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebotTest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
