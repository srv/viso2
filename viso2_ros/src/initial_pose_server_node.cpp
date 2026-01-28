/**
 *  @file initial_pose_server_node.cpp
 *  @brief Launch the initial_pose_server
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_pose_server_node");
    ros::start();

    InitialPoseServer node;

    ros::spin();

    return 0;
}