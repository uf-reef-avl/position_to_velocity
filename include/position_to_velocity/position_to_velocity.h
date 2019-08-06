//
// Created by prashant on 3/1/19.
//

#ifndef PROJECT_POSITION_TO_VELOCITY_H
#define PROJECT_POSITION_TO_VELOCITY_H

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>
#include <reef_msgs/matrix_operation.h>
#include <reef_msgs/dynamics.h>
#include <tf2_eigen/tf2_eigen.h>
#include <random>


namespace reef_estimator
{
class PoseToVelocity {
 private:
  ros::NodeHandle nh_private_;
  ros::NodeHandle nh_;

  ros::Publisher velocity_ned_pub_;
  ros::Publisher twist_ned_pub_;
  ros::Publisher velocity_camera_frame_pub_;
  ros::Publisher velocity_body_level_pub_;

  ros::Subscriber pose_stamped_subs_;
  ros::Subscriber nav_odom_subs_;
  ros::Subscriber transform_stamped_subs_;

  void truth_callback(const geometry_msgs::PoseStampedConstPtr &msg);
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);
  void transform_callback(const geometry_msgs::TransformStampedConstPtr &msg);
  void process_msg(Eigen::Affine3d& pose_msg, double& msg_time);

  geometry_msgs::TwistWithCovarianceStamped vel_cov_msg;
  geometry_msgs::TwistStamped vel_msg;


  bool convert_to_ned_;
  bool initialized_;
  bool verbose_;

  double alpha;
  double DT;
  double yaw;
  double rate;
  double previous_time;
  double x_vel_covariance;
  double y_vel_covariance;

  Eigen::Affine3d previous_pose;
  Eigen::Affine3d velocity_current;
  Eigen::Affine3d velocity_previous;
  Eigen::Affine3d body_to_camera;
  Eigen::Affine3d filtered_velocity_NED;
  Eigen::Affine3d filtered_velocity_camera_frame;
  Eigen::Affine3d filtered_velocity_body_leveled_frame;

  Eigen::Matrix3d C_from_NED_to_body_level;
  Eigen::Matrix3d covariance_ned_frame;
  Eigen::Matrix3d covariance_camera_frame;
  Eigen::Matrix3d covariance_body_level_frame;
  Eigen::Vector3d noise_vector;

  void load_msg(const Eigen::Vector3d& vec_in, geometry_msgs::Vector3& vec_out);

  int rand_numb;
  std::mt19937 engine;
  std::normal_distribution<double> distribution;
  double mocap_noise_std;

 public:
  PoseToVelocity();
  ~PoseToVelocity();

};
}

#endif //PROJECT_POSITION_TO_VELOCITY_H
