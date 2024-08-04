/*
    pidtuner.cpp
    PID Tuner Node
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

//
// Standard library
//

#include <string>
#include <vector>

//
// ROS
//

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
#include <angles/angles.h>
#include <urdf/model.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const int SPIN_RATE = 50;

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void configure(ros::NodeHandle node)
{
}

void initialize(ros::NodeHandle node)
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pidtuner");
  ros::NodeHandle node;

  configure(node);
  initialize(node);

  ros::Rate rate(SPIN_RATE);

  while(node.ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}