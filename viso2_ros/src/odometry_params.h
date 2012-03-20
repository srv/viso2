#include <ros/ros.h>

#include <viso_stereo.h>
#include <viso_mono.h>

namespace viso2_ros
{

namespace odometry_params
{
 
/// loads matcher params
void loadParams(const ros::NodeHandle& local_nh, Matcher::parameters& params)
{
  local_nh.getParam("nms_n",                  params.nms_n);
  local_nh.getParam("nms_tau",                params.nms_tau);
  local_nh.getParam("match_binsize",          params.match_binsize);
  local_nh.getParam("match_radius",           params.match_radius);
  local_nh.getParam("match_disp_tolerance",   params.match_disp_tolerance);
  local_nh.getParam("outlier_disp_tolerance", params.outlier_disp_tolerance);
  local_nh.getParam("outlier_flow_tolerance", params.outlier_flow_tolerance);
  local_nh.getParam("multi_stage",            params.multi_stage);
  local_nh.getParam("half_resolution",        params.half_resolution);
  local_nh.getParam("refinement",             params.refinement);
}

/// loads bucketing params
void loadParams(const ros::NodeHandle& local_nh, VisualOdometry::bucketing& bucketing)
{
  local_nh.getParam("max_features",  bucketing.max_features);
  local_nh.getParam("bucket_width",  bucketing.bucket_width);
  local_nh.getParam("bucket_height", bucketing.bucket_height);
}

/// loads general odometry params
void loadCommonParams(const ros::NodeHandle& local_nh, VisualOdometry::parameters& params)
{
  loadParams(local_nh, params.match);
  loadParams(local_nh, params.bucket);
}

/// loads stereo specific params
void loadParams(const ros::NodeHandle& local_nh, VisualOdometryStereo::parameters& params)
{
  local_nh.getParam("ransac_iters",     params.ransac_iters);
  local_nh.getParam("inlier_threshold", params.inlier_threshold);
  local_nh.getParam("reweighting",      params.reweighting);
  loadCommonParams(local_nh, params);
}

/// loads mono specific params
void loadParams(const ros::NodeHandle& local_nh, VisualOdometryMono::parameters& params)
{
  if (!local_nh.getParam("camera_height", params.height))
  {
    ROS_WARN("Parameter 'camera_height' is required but not set. Using default: %f", params.height);
  }
  if (!local_nh.getParam("camera_pitch", params.pitch))
  {
    ROS_WARN("Paramter 'camera_pitch' is required but not set. Using default: %f", params.pitch);
  }
  local_nh.getParam("ransac_iters",     params.ransac_iters);
  local_nh.getParam("inlier_threshold", params.inlier_threshold);
  local_nh.getParam("motion_threshold", params.motion_threshold);
  loadCommonParams(local_nh, params);
}

} // end of namespace

} // end of namespace

