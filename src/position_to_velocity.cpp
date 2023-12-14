//
// Created by prashant on 3/1/19.
//

#include "position_to_velocity/position_to_velocity.h"

#define DBG(msg, ...) { if (verbose_) ROS_WARN_STREAM((msg), ##__VA_ARGS__);}

namespace reef_estimator
{
  PoseToVelocity::PoseToVelocity() :
  nh_(),
  nh_private_("~"),
  DT(0),
  initialized_(false)
  {
      nh_private_.param<bool>("convert_to_ned", convert_to_ned_, false );
      nh_private_.param<bool>("verbose", verbose_, false );
      nh_private_.param<double>("alpha",alpha,1);
      nh_private_.param<double>("mocap_noise_std", mocap_noise_std, 0.001);
      nh_private_.param<double>("update_rate", rate, 100);
      distribution =  std::normal_distribution<>(0,mocap_noise_std);
      rand_numb = rand() % 100;
      std::mt19937 engine(rand_numb);


      pose_stamped_subs_ = nh_.subscribe<geometry_msgs::PoseStamped>("pose_stamped",1 , &PoseToVelocity::truth_callback, this);
      nav_odom_subs_ = nh_.subscribe<nav_msgs::Odometry>("odom",1 , &PoseToVelocity::odom_callback, this);
      transform_stamped_subs_ = nh_.subscribe<geometry_msgs::TransformStamped>("transform_stamped",1 , &PoseToVelocity::transform_callback, this);

      velocity_ned_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity/ned",1);
      twist_ned_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("velocity",1);
      velocity_camera_frame_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity/camera_frame",1);
      velocity_body_level_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity/body_level_frame",1);

      reef_msgs::loadTransform("body_to_camera",body_to_camera);

      ROS_WARN_STREAM("Body To camera Rotation Matrix");
      ROS_WARN_STREAM(body_to_camera.linear());
      ROS_WARN_STREAM("Body To camera Translation");
      ROS_WARN_STREAM(body_to_camera.translation());

  }

  PoseToVelocity::~PoseToVelocity() {

  }

    void PoseToVelocity::truth_callback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        Eigen::Affine3d pose_msg;
        tf2::fromMsg(msg->pose, pose_msg);

        double msg_time_sec = msg->header.stamp.toSec();
        vel_cov_msg.header.stamp = msg->header.stamp;

        if(convert_to_ned_)
          pose_msg = reef_msgs::convertNWU2NED(pose_msg);

        process_msg(pose_msg, msg_time_sec);

    }

    void PoseToVelocity::transform_callback(const geometry_msgs::TransformStampedConstPtr &msg)
    {
        Eigen::Affine3d pose_msg;
        pose_msg = tf2::transformToEigen(*msg);

        double msg_time_sec = msg->header.stamp.toSec();
        vel_cov_msg.header.stamp = msg->header.stamp;

        if(convert_to_ned_)
            pose_msg = reef_msgs::convertNWU2NED(pose_msg);

        process_msg(pose_msg, msg_time_sec);
    }


    void PoseToVelocity::odom_callback(const nav_msgs::OdometryConstPtr &msg)
    {
        Eigen::Affine3d pose_msg;
        tf2::fromMsg(msg->pose.pose, pose_msg);

        double msg_time_sec = msg->header.stamp.toSec();
        vel_cov_msg.header.stamp = msg->header.stamp;

        if(convert_to_ned_)
            pose_msg = reef_msgs::convertNWU2NED(pose_msg);

        process_msg(pose_msg, msg_time_sec);

    }


  void PoseToVelocity::process_msg(Eigen::Affine3d& current_pose, double& current_time) {

    if(!initialized_)
    {
      previous_time = current_time;
      previous_pose = current_pose;
      initialized_ = true;
      velocity_previous.translation() = Eigen::Vector3d(0 , 0, 0);
      return;
    }

    DT = current_time - previous_time;
    DBG("DT")
    DBG(DT);
    if(DT >= 1.0/rate)
    {
      noise_vector = Eigen::Vector3d(distribution(engine),distribution(engine),0.0);

      if(mocap_noise_std>0){
        current_pose.translation() = current_pose.translation() + noise_vector;
        x_vel_covariance  = (pow(mocap_noise_std,2) + pow(mocap_noise_std,2))/(DT*DT);
        y_vel_covariance = (pow(mocap_noise_std,2) + pow(mocap_noise_std,2))/(DT*DT);
      }
      else{
        x_vel_covariance  = (pow(0.01,2) + pow(0.01,2))/(DT*DT);
        y_vel_covariance = (pow(0.01,2) + pow(0.01,2))/(DT*DT);
      }

      reef_msgs::get_yaw(current_pose.linear().transpose(), yaw);
      C_from_NED_to_body_level = reef_msgs::roll_pitch_yaw_to_rotation_321(0.0, 0.0, yaw);
      DBG("Body level rotation matrix");
      DBG(C_from_NED_to_body_level);

      velocity_current.linear() = (current_pose.linear() * previous_pose.linear().transpose());
      velocity_current.translation() = (current_pose.translation() - previous_pose.translation()) / DT;

      DBG("Unfiltered Velocity NED Frame");
      DBG(velocity_current.translation());

      filtered_velocity_NED.translation() = alpha * velocity_current.translation() + (1-alpha) * velocity_previous.translation();

      DBG("Filtered Velocity NED");
      DBG(filtered_velocity_NED.translation());

      Eigen::Vector3d angular_velocity;
      Eigen::Quaterniond delta_quat;
      delta_quat = velocity_current.linear();
      double euler_angle = 2 * atan2(delta_quat.vec().norm(), delta_quat.w());
      angular_velocity = delta_quat.vec()/delta_quat.vec().norm() *  (euler_angle / DT);

      covariance_ned_frame.diagonal() << x_vel_covariance, y_vel_covariance, 0.0;
      vel_cov_msg.twist.covariance[0] = x_vel_covariance;
      vel_cov_msg.twist.covariance[7] = y_vel_covariance;
      load_msg(filtered_velocity_NED.translation(), vel_cov_msg.twist.twist.linear);
      load_msg(angular_velocity, vel_cov_msg.twist.twist.angular);
      load_msg(filtered_velocity_NED.translation(), vel_msg.twist.linear);
      load_msg(angular_velocity, vel_msg.twist.angular);

      vel_cov_msg.header.stamp = ros::Time(current_time);
      vel_msg.header.stamp = ros::Time(current_time);

      velocity_ned_pub_.publish(vel_cov_msg);
      twist_ned_pub_.publish(vel_msg);

      filtered_velocity_camera_frame.translation() = body_to_camera.linear().transpose() * current_pose.linear().transpose() * filtered_velocity_NED.translation();
      covariance_camera_frame = body_to_camera.linear().transpose() * current_pose.linear().transpose() * covariance_ned_frame * current_pose.linear() * body_to_camera.linear();
      vel_cov_msg.twist.covariance[0] = covariance_camera_frame(0,0);
      vel_cov_msg.twist.covariance[7] = covariance_camera_frame(1,1);
      load_msg(filtered_velocity_camera_frame.translation(), vel_cov_msg.twist.twist.linear);
      velocity_camera_frame_pub_.publish(vel_cov_msg);

      DBG("Filtered Velocity Camera Frame");
      DBG(filtered_velocity_camera_frame.translation());

      filtered_velocity_body_leveled_frame.translation() =  C_from_NED_to_body_level * filtered_velocity_NED.translation();
      covariance_body_level_frame = C_from_NED_to_body_level * covariance_ned_frame * C_from_NED_to_body_level.transpose();
      vel_cov_msg.twist.covariance[0] = covariance_body_level_frame(0,0);
      vel_cov_msg.twist.covariance[7] = covariance_body_level_frame(1,1);
      load_msg(filtered_velocity_body_leveled_frame.translation(), vel_cov_msg.twist.twist.linear);
      velocity_body_level_pub_.publish(vel_cov_msg);
      DBG("Filtered Velocity Body Level Frame");
      DBG(filtered_velocity_body_leveled_frame.translation());

      previous_pose = current_pose;
      previous_time = current_time;
      velocity_previous = filtered_velocity_NED;
    }
  }

void PoseToVelocity::load_msg(const Eigen::Vector3d &vec_in, geometry_msgs::Vector3& vec_out)
{
    DBG("Inside load_msg function");
  vec_out.x = vec_in(0);
  vec_out.y = vec_in(1);
  vec_out.z = vec_in(2);
}

}
