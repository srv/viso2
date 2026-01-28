/**
 *  @file initial_pose_server.cpp
 *  @brief Obtain the initial pose from .launch, .yaml or tfs 
 *         and sends it to the node that requests it.
 *  @package viso2_ros
 *  @project viso2_ros
 *
 *  @author
 *    Bo Miquel Nordfeldt-Fiol (2026)
 *
 *  @license BSD-4-Clause
 *    This file is part of the viso2_ros project and is released under
 *    the GPL License. See the LICENSE file for details.
 */


#include "initial_pose_server.hpp"


InitialPoseServer::InitialPoseServer(): nh_{}, nhp_{"~"}, tf_listener_{tf_buffer_}
{
    // Get parameters.
    double tx, ty, tz, qx, qy, qz, qw;
    nhp_.param<double>("tx", tx, 0.0);
    nhp_.param<double>("ty", ty, 0.0);
    nhp_.param<double>("tz", tz, 0.0);
    nhp_.param<double>("qx", qx, 0.0);
    nhp_.param<double>("qy", qy, 0.0);
    nhp_.param<double>("qz", qz, 0.0);
    nhp_.param<double>("qw", qw, 1.0);
    nhp_.param<std::string>("world_frame", world_frame_, "world_ned");
    nhp_.param<std::string>("robot_frame", robot_frame_, "base_link");

    // Build transform derived from the parameters.
    tf2::Transform tf_aux;
    tf_aux.setOrigin(tf2::Vector3(tx, ty, tz));
    tf_aux.setRotation(tf2::Quaternion(qx, qy, qz, qw));
    param_tf_.transform = tf2::toMsg(tf_aux);
    param_tf_.header.frame_id = world_frame_;
    param_tf_.child_frame_id = robot_frame_;

    // Set server.
    i_pose_server_ = nh_.advertiseService("get_pose", &InitialPoseServer::getInitialPose, this);

    ROS_INFO_STREAM("[IPS:] Ready to send punctual pose.");
}


bool InitialPoseServer::getInitialPose(viso2_ros::GetTransform::Request& req,
                                       viso2_ros::GetTransform::Response& res)
{
    ros::Time query_time = req.timestamp; 
    if (query_time.isZero())
        query_time = ros::Time(0);

    try
    {
        res.tf = tf_buffer_.lookupTransform(world_frame_,
                                            robot_frame_,
                                            query_time,
                                            ros::Duration(0.1));
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
        res.tf = param_tf_;
        if (query_time.isZero() || query_time == ros::Time(0))
            res.tf.header.stamp = ros::Time::now();
        else
            res.tf.header.stamp = query_time;
    }

    return true;    
}