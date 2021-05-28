
#ifndef ODOMETER_BASE_H_
#define ODOMETER_BASE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>

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
  bool invert_tf_;

  // the current integrated camera pose
  tf::Transform integrated_pose_;
  // timestamp of the last update
  ros::Time last_update_time_;

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
    local_nh.param("invert_tf", invert_tf_, false);

    ROS_INFO_STREAM("Basic Odometer Settings:" << std::endl <<
                    "  odom_frame_id      = " << odom_frame_id_ << std::endl <<
                    "  base_link_frame_id = " << base_link_frame_id_ << std::endl <<
                    "  sensor_frame_id    = " << sensor_frame_id_ << std::endl <<
                    "  publish_tf         = " << (publish_tf_?"true":"false") << std::endl <<
                    "  invert_tf          = " << (invert_tf_?"true":"false"));

    // advertise
    odom_pub_ = local_nh.advertise<nav_msgs::Odometry>("odometry", 1);
    pose_pub_ = local_nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    reset_service_ = local_nh.advertiseService("reset_pose", &OdometerBase::resetPose, this);


    // [ INFO] [1622114609.566293356, 1614260485.775736583]: base_to_sensor x: 0.399952
    // [ INFO] [1622114609.566315631, 1614260485.775736583]: base_to_sensor y: -0.060000
    // [ INFO] [1622114609.566333835, 1614260485.775736583]: base_to_sensor z: 0.800000
    // [ INFO] [1622114609.566370158, 1614260485.775736583]: base_to_sensor roll: 0.000000
    // [ INFO] [1622114609.566387227, 1614260485.775736583]: base_to_sensor pitch: -0.000000
    // [ INFO] [1622114609.566404184, 1614260485.775736583]: base_to_sensor yaw: 1.570000


    tf::Transform base_to_sensor_auxiliar ;

    tf::Vector3 aux_t(0.399952, -0.060000, 0.800000) ;

    tf::Quaternion myQuaternion;
    myQuaternion.setRPY(0.000000, -0.000000, 1.570000) ;

    base_to_sensor_auxiliar.setOrigin(aux_t) ;

    base_to_sensor_auxiliar.setRotation(myQuaternion) ;

    // // Precompute sine/cosine
    // double s_roll = sin(-0.06352688364046247);
    // double c_roll = cos(-0.06352688364046247);
    // double s_pitch = sin(-0.1750601524898814);
    // double c_pitch = cos(-0.1750601524898814);
    // double s_yaw = sin(1.2347373125385348);
    // double c_yaw = cos(1.2347373125385348);

    // // Compute rotation matrix
    // tf::Matrix3x3 rot_mat(
    // c_yaw * c_pitch, -s_yaw * c_roll + c_yaw * s_pitch * s_roll , s_yaw * s_roll + c_yaw * s_pitch * c_roll,
    // s_yaw * c_pitch, c_yaw * c_roll + s_yaw * s_pitch * s_roll, -c_yaw * s_roll + s_yaw * s_pitch * c_roll,
    // -s_pitch, c_pitch * s_roll, c_pitch * c_roll);

    tf::Quaternion myQuaternion2(0.02478365567, -0.08955947924, 0.5741207366, 0.8134803316) ;
    
    tf::Vector3 t(74.568675009499998, 18.5392068604, 5.4271955410699997) ;

    integrated_pose_.setOrigin(t) ;
    integrated_pose_.setRotation(myQuaternion2) ;

    integrated_pose_ = base_to_sensor_auxiliar.inverse() * integrated_pose_ * base_to_sensor_auxiliar;

    // O aquesta?
    // integrated_pose_.setIdentity();

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
     
    /* Transform integrated pose to base frame. The program listen tf topic (?) and obtain the transform between
    base link and the sensor. If the transfrom doesn't exist the program advise you.  */
    tf::StampedTransform base_to_sensor;
    std::string error_msg;
    if (tf_listener_.canTransform(base_link_frame_id_, sensor_frame_id_, timestamp, &error_msg))
    {
      tf_listener_.lookupTransform(
          base_link_frame_id_,
          sensor_frame_id_,
          timestamp, base_to_sensor);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
                              "will assume it as identity!",
                              base_link_frame_id_.c_str(),
                              sensor_frame_id_.c_str());
      ROS_DEBUG("Transform error: %s", error_msg.c_str());
      base_to_sensor.setIdentity();
    }

    // tf::Vector3 aux6(integrated_pose_.getOrigin()) ;

    // ROS_INFO("integrated_pose_ x: %f", aux6.getX()) ;

    // tf::Vector3 aux5(base_to_sensor.getOrigin()) ;

    // ROS_INFO("base_to_sensor x: %f", aux5.getX()) ;
    // ROS_INFO("base_to_sensor y: %f", aux5.getY()) ;
    // ROS_INFO("base_to_sensor z: %f", aux5.getZ()) ;

    // tf::Quaternion myQuaternion = base_to_sensor.getRotation() ;

    /**< quaternion -> rotation Matrix */
    // tf::Matrix3x3 m(myQuaternion);
  
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
  
    // ROS_INFO("base_to_sensor roll: %f", roll) ;
    // ROS_INFO("base_to_sensor pitch: %f", pitch) ;
    // ROS_INFO("base_to_sensor yaw: %f", yaw) ;

    // tf::Quaternion q_aux(0.02478365567, -0.08955947924, 0.5741207366, 0.8134803316);
    
    // tf::Vector3 t_aux(74.568675009499998, 18.5392068604, 5.4271955410699997) ;

    // tf::Transform TF_aux ;

    // TF_aux.setOrigin(t_aux) ;
    // TF_aux.setRotation(q_aux) ;

    /* Multiplication of tfs. */
    tf::Transform base_transform = base_to_sensor * integrated_pose_ * base_to_sensor.inverse();

    // tf::Transform base_transform = base_to_sensor * integrated_pose_ * base_to_sensor.inverse() * TF_aux;

    // tf::Vector3 aux4(base_transform.getOrigin()) ;

    // ROS_INFO("Matriu despres de mutiplicar-se amb base_to_sensor x: %f", aux4.getX()) ;

    /* */
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = timestamp;
    odometry_msg.header.frame_id = odom_frame_id_;
    odometry_msg.child_frame_id = base_link_frame_id_;
    tf::poseTFToMsg(base_transform, odometry_msg.pose.pose);

    /* Calculate twist (not possible for first run as no delta_t can be computed). Is not the same multiplication? */
    tf::Transform delta_base_transform = base_to_sensor * delta_transform * base_to_sensor.inverse();
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
      if (invert_tf_)
      {
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(base_transform.inverse(), timestamp,
	    base_link_frame_id_, odom_frame_id_));
      }
      else
      {
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(base_transform, timestamp,
            odom_frame_id_, base_link_frame_id_));
      }
    }

    last_update_time_ = timestamp;
  }


  bool resetPose(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    integrated_pose_.setIdentity();
    return true;
  }

};

} // end of namespace

#endif

