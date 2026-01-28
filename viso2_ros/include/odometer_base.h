
#ifndef ODOMETER_BASE_H_
#define ODOMETER_BASE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

  // Publisher
  ros::Publisher odom_pub_;
  ros::Publisher pose_pub_;
  ros::ServiceServer reset_service_;

  // tf2 related
  std::string odom_frame_id_;
  std::string sensor_frame_id_;
  std::string base_link_frame_id_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  bool publish_tf_;
  bool invert_tf_;

  // The current integrated camera pose
  tf2::Transform integrated_pose_;

  // Timestamp of the last update
  ros::Time last_update_time_;

  // Initial pose related.
  bool initial_pose_in_camera_frame_;

  // covariances
  boost::array<double, 36> pose_covariance_;
  boost::array<double, 36> twist_covariance_;

public:

  OdometerBase(): tf_listener_(tf2_buffer_)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    local_nh.param("odom_frame_id", odom_frame_id_, std::string("/odom"));
    local_nh.param("base_link_frame_id", base_link_frame_id_, std::string("/base_link"));
    local_nh.param("sensor_frame_id", sensor_frame_id_, std::string("/camera"));
    local_nh.param("publish_tf", publish_tf_, true);
    local_nh.param("invert_tf", invert_tf_, false);
    local_nh.param("initial_pose_in_camera_frame", initial_pose_in_camera_frame_, false);
    ROS_INFO_STREAM("Basic Odometer Settings:" << std::endl <<
                    "  odom_frame_id                = " << odom_frame_id_ << std::endl <<
                    "  base_link_frame_id           = " << base_link_frame_id_ << std::endl <<
                    "  publish_tf                   = " << (publish_tf_?"true":"false") << std::endl <<
                    "  invert_tf                    = " << (invert_tf_?"true":"false") << std::endl <<
                    "  initial_pose_in_camera_frame = " << (initial_pose_in_camera_frame_?"true":"false"));

    // Publishers.
    odom_pub_ = local_nh.advertise<nav_msgs::Odometry>("odometry", 1);
    pose_pub_ = local_nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    // Service.
    reset_service_ = local_nh.advertiseService("reset_pose", &OdometerBase::resetPose, this);

    // Init pose and covariance.
    integrated_pose_.setIdentity();
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

  void integrateAndPublish(const tf2::Transform& delta_transform, const ros::Time& timestamp)
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
    }
    // {wo}^T_{c_camera} = {wo}^T_{p_camera} * {p_camera}^T_{c_camera}
    integrated_pose_ *= delta_transform;

    // Transform integrated pose to base frame
    tf2::Transform base_to_sensor;
    base_to_sensor.setIdentity();
    try
    {
      geometry_msgs::TransformStamped ts = tf2_buffer_.lookupTransform(base_link_frame_id_, sensor_frame_id_, timestamp, ros::Duration(0.1));
      tf2::fromMsg(ts.transform, base_to_sensor);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
                              "will assume it as identity! Error: %s",
                              base_link_frame_id_.c_str(),
                              sensor_frame_id_.c_str(), ex.what());
    }
    
    // Integrate.
    tf2::Transform base_transform;
    if (initial_pose_in_camera_frame_)
    {
      base_transform = base_to_sensor * integrated_pose_ * base_to_sensor.inverse();
    }
    else
    {
      // {wo}^T_{c_robot} = {wo}^T_{c_camera} * {camera}^T_{robot}
      base_transform = integrated_pose_ * base_to_sensor.inverse();
    }

    // Create message.
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = timestamp;
    odometry_msg.header.frame_id = odom_frame_id_;
    odometry_msg.child_frame_id = base_link_frame_id_;

    // Get pose.
    tf2::toMsg(base_transform, odometry_msg.pose.pose);

    // Calculate twist (not possible for first run as no delta_t can be computed)
    // {p_robot}^T_{c_robot} = {robot}^T_{camera} * {p_camera}^T_{c_camera} * {camera}^T_{robot}.
    tf2::Transform delta_base_transform = base_to_sensor * delta_transform * base_to_sensor.inverse();
    if (!last_update_time_.isZero())
    {
      double delta_t = (timestamp - last_update_time_).toSec();
      if (delta_t > 0)
      {
        odometry_msg.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / delta_t;
        odometry_msg.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / delta_t;
        odometry_msg.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / delta_t;
        tf2::Quaternion delta_rot = delta_base_transform.getRotation();
        double angle = delta_rot.getAngle();
        tf2::Vector3 axis = delta_rot.getAxis();
        tf2::Vector3 angular_twist = axis * angle / delta_t;
        odometry_msg.twist.twist.angular.x = angular_twist.x();
        odometry_msg.twist.twist.angular.y = angular_twist.y();
        odometry_msg.twist.twist.angular.z = angular_twist.z();
      }
    }

    odometry_msg.pose.covariance = pose_covariance_;
    odometry_msg.twist.covariance = twist_covariance_;
    odom_pub_.publish(odometry_msg);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = odometry_msg.header;
    pose_msg.pose = odometry_msg.pose.pose;

    pose_pub_.publish(pose_msg);

    if (publish_tf_)
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = timestamp;

      if (invert_tf_)
      {
        transformStamped.header.frame_id = base_link_frame_id_;
        transformStamped.child_frame_id = odom_frame_id_;
        transformStamped.transform = tf2::toMsg(base_transform.inverse());
      }
      else
      {
        transformStamped.header.frame_id = odom_frame_id_;
        transformStamped.child_frame_id = base_link_frame_id_;
        transformStamped.transform = tf2::toMsg(base_transform);
      }
      tf_broadcaster_.sendTransform(transformStamped);
    }

    last_update_time_ = timestamp;
  }

  bool resetPose(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    integrated_pose_.setIdentity();
    return true;
  }

  void setPose(const tf2::Transform& pose) 
  {
    if (initial_pose_in_camera_frame_)
    {
      integrated_pose_ = pose;
    }
    else
    {
      tf2::Transform base_to_sensor;
      base_to_sensor.setIdentity();
      try
      {
        geometry_msgs::TransformStamped ts = tf2_buffer_.lookupTransform(base_link_frame_id_, sensor_frame_id_, ros::Time(0), ros::Duration(0.1));
        tf2::fromMsg(ts.transform, base_to_sensor);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
                                "will assume it as identity! Error: %s",
                                base_link_frame_id_.c_str(),
                                sensor_frame_id_.c_str(), ex.what());
      }
      ROS_WARN_STREAM("[VO->integrateAndPublish:] base_to_sensor: " << std::endl <<
                                                             "tx: " << base_to_sensor.getOrigin().getX() << std::endl << 
                                                             "ty: " << base_to_sensor.getOrigin().getY() << std::endl << 
                                                             "tz: " << base_to_sensor.getOrigin().getZ() << std::endl << 
                                                             "qx: " << base_to_sensor.getRotation().getX() << std::endl << 
                                                             "qy: " << base_to_sensor.getRotation().getY() << std::endl << 
                                                             "qz: " << base_to_sensor.getRotation().getZ() << std::endl << 
                                                             "qw: " << base_to_sensor.getRotation().getW());
      // {wo}^T_{c_camera} = {wo}^T_{c_robot} * {robot}^T_{camera}
      integrated_pose_ = pose * base_to_sensor;
    }
  }

};

} // end of namespace

#endif

