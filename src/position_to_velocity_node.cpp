//
// Created by prashant on 3/1/19.
//

#include <ros/ros.h>
#include "position_to_velocity/position_to_velocity.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_to_velocity_node");
  reef_estimator::PoseToVelocity object;
  ros::spin();
  return 0;
}