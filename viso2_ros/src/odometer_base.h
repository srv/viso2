
#ifndef ODOMETER_BASE_H_
#define ODOMETER_BASE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace viso2_ros
{

/**
 * Base class for odometers, handles tf's, odometry and pose
 * publishing. This can be used as base for any incremental pose estimating
 * sensor. Sensors that measure velocities cannot be used.
 */
class OdometerBase
{

private:

  // publisher
  ros::Publisher odom_pub_;
  ros::Publisher pose_pub_;

  ros::ServiceServer reset_service_;

  // tf related
  std::string sensor_frame_id_;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  bool publish_tf_;

  // the current integrated camera pose
  tf::Transform integrated_pose_;
  // timestamp of the last update
  ros::Time last_update_time_;
  // the last obtained base to sensor transform
  tf::StampedTransform base_to_sensor_;

  // covariances
  boost::array<double, 36> pose_covariance_;
  boost::array<double, 36> twist_covariance_;

public:

  OdometerBase()
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");

    local_nh.param("odom_frame_id", odom_frame_id_, std::string("/odom"));
    local_nh.param("base_link_frame_id", base_link_frame_id_, std::string("/base_link"));
    local_nh.param("sensor_frame_id", sensor_frame_id_, std::string("/camera"));
    local_nh.param("publish_tf", publish_tf_, true);

    ROS_INFO_STREAM("Basic Odometer Settings:" << std::endl <<
                    "  odom_frame_id      = " << odom_frame_id_ << std::endl <<
                    "  base_link_frame_id = " << base_link_frame_id_ << std::endl <<
                    "  publish_tf         = " << (publish_tf_?"true":"false"));
    
    // advertise
    odom_pub_ = local_nh.advertise<nav_msgs::Odometry>("odometry", 1);
    pose_pub_ = local_nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    reset_service_ = local_nh.advertiseService("reset_pose", &OdometerBase::resetPose, this);

    integrated_pose_.setIdentity();
    base_to_sensor_.setIdentity();

    pose_covariance_.assign(0.0);
    twist_covariance_.assign(0.0);
  }

protected:

  void setSensorFrameId(const std::string& frame_id)
  {
    sensor_frame_id_ = frame_id;
  }

  std::string getSensorFrameId() const
  {
    return sensor_frame_id_;
  }

  void setPoseCovariance(const boost::array<double, 36>& pose_covariance)
  {
    pose_covariance_ = pose_covariance;
  }

  void setTwistCovariance(const boost::array<double, 36>& twist_covariance)
  {
    twist_covariance_ = twist_covariance;
  }

  void integrateAndPublish(const tf::Transform& delta_transform, const ros::Time& timestamp)
  {
    if (sensor_frame_id_.empty())
    {
      ROS_ERROR("[odometer] update called with unknown sensor frame id!");
      return;
    }
    if (timestamp < last_update_time_)
    {
      ROS_WARN("[odometer] saw negative time change in incoming sensor data, resetting pose.");
      integrated_pose_.setIdentity();
      tf_listener_.clear();
    }
    integrated_pose_ *= delta_transform;

    // transform integrated pose to base frame
    updateBaseToSensorTf(timestamp);
    tf::Transform base_transform = base_to_sensor_ * integrated_pose_ * base_to_sensor_.inverse();

    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = timestamp;
    odometry_msg.header.frame_id = odom_frame_id_;
    odometry_msg.child_frame_id = base_link_frame_id_;
    tf::poseTFToMsg(base_transform, odometry_msg.pose.pose);

    // calculate twist (not possible for first run as no delta_t can be computed)
    tf::Transform delta_base_transform = base_to_sensor_ * delta_transform * base_to_sensor_.inverse();
    if (!last_update_time_.isZero())
    {
      double delta_t = (timestamp - last_update_time_).toSec();
      if (delta_t)
      {
        odometry_msg.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / delta_t;
        odometry_msg.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / delta_t;
        odometry_msg.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / delta_t;
        tf::Quaternion delta_rot = delta_base_transform.getRotation();
        tfScalar angle = delta_rot.getAngle();
        tf::Vector3 axis = delta_rot.getAxis();
        tf::Vector3 angular_twist = axis * angle / delta_t;
        odometry_msg.twist.twist.angular.x = angular_twist.x();
        odometry_msg.twist.twist.angular.y = angular_twist.y();
        odometry_msg.twist.twist.angular.z = angular_twist.z();
      }
    }

    odometry_msg.pose.covariance = pose_covariance_;
    odometry_msg.twist.covariance = twist_covariance_;
    odom_pub_.publish(odometry_msg);
    
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = odometry_msg.header.stamp;
    pose_msg.header.frame_id = odometry_msg.header.frame_id;
    pose_msg.pose = odometry_msg.pose.pose;

    pose_pub_.publish(pose_msg);

    if (publish_tf_)
    {
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(base_transform, timestamp,
          odom_frame_id_, base_link_frame_id_));
    }

    last_update_time_ = timestamp;
  }


  bool resetPose(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    integrated_pose_.setIdentity();
    return true;
  }

private:
  void updateBaseToSensorTf(const ros::Time& timestamp)
  {
    std::string error_msg;
    if (tf_listener_.canTransform(base_link_frame_id_, sensor_frame_id_, timestamp, &error_msg))
    {
      tf_listener_.lookupTransform(base_link_frame_id_,
          sensor_frame_id_,
          timestamp,
          base_to_sensor_);
    }
    else if (tf_listener_.canTransform(base_link_frame_id_, sensor_frame_id_, ros::Time(0), &error_msg))
    {
      ROS_WARN_THROTTLE(10.0, "Using the outdated transform from '%s' to '%s'",
                        base_link_frame_id_.c_str(), sensor_frame_id_.c_str());
      ROS_DEBUG("Transform error: %s", error_msg.c_str());
      tf_listener_.lookupTransform(base_link_frame_id_,
          sensor_frame_id_,
          ros::Time(0),
          base_to_sensor_);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available",
                        base_link_frame_id_.c_str(), sensor_frame_id_.c_str());
      ROS_DEBUG("Transform error: %s", error_msg.c_str());
    }
  }
};

} // end of namespace

#endif

