#include <ros/ros.h>

#include <viso_stereo.h>
#include <viso_mono.h>
#include <viso_mono_omnidirectional.h>

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

/// loads common odometry params
void loadCommonParams(const ros::NodeHandle& local_nh, VisualOdometry::parameters& params)
{
  loadParams(local_nh, params.match);
  loadParams(local_nh, params.bucket);
}

/// loads common & stereo specific params
void loadParams(const ros::NodeHandle& local_nh, VisualOdometryStereo::parameters& params)
{
  loadCommonParams(local_nh, params);
  local_nh.getParam("ransac_iters",     params.ransac_iters);
  local_nh.getParam("inlier_threshold", params.inlier_threshold);
  local_nh.getParam("reweighting",      params.reweighting);
}

/// loads common & mono specific params
void loadParams(const ros::NodeHandle& local_nh, VisualOdometryMono::parameters& params)
{
  loadCommonParams(local_nh, params);
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
}

/// loads common & omnidirectional mono specific params
void loadParams(const ros::NodeHandle& local_nh, VisualOdometryMonoOmnidirectional::parameters& params)
{
  loadCommonParams(local_nh, params);
  local_nh.getParam("ransac_iters",     params.ransac_iters);
  local_nh.getParam("inlier_threshold", params.inlier_threshold);
  local_nh.getParam("motion_threshold", params.motion_threshold);

  std::string path;
  local_nh.getParam("calib_path", path);
  std::ifstream file(path.c_str());
  if(file.is_open())
  {
    std::string s;

    //Read polynomial coefficients
    std::getline(file, s);
    file >> params.omnidirectional_calib.length_pol;
    for (int i = 0; i < params.omnidirectional_calib.length_pol; i++)
    {
      file >> params.omnidirectional_calib.pol[i];
    }

    //Read inverse polynomial coefficients
    std::getline(file, s);
    std::getline(file, s);
    std::getline(file, s);
    file >> params.omnidirectional_calib.length_invpol;
    for (int i = 0; i < params.omnidirectional_calib.length_invpol; i++)
    {
      file >> params.omnidirectional_calib.invpol[i];
    }

    //Read center coordinates
    std::getline(file, s);
    std::getline(file, s);
    std::getline(file, s);
    file >> params.omnidirectional_calib.xc >> 
            params.omnidirectional_calib.yc;
    
    //Read affine coefficients
    std::getline(file, s);
    std::getline(file, s);
    std::getline(file, s);
    file >> params.omnidirectional_calib.c >> 
            params.omnidirectional_calib.d >>
            params.omnidirectional_calib.e;
    
    //Read image size
    std::getline(file, s);
    std::getline(file, s);
    std::getline(file, s);
    file >> params.omnidirectional_calib.height >> 
            params.omnidirectional_calib.width;

    file.close();
  }
  else
  {
    std::cout << "File not found or path not provided" << std::endl;
    std::cout << "Using default parameters" << std::endl;
    params.omnidirectional_calib.fx = 1;
    params.omnidirectional_calib.fy = 1;
    params.omnidirectional_calib.cx = 0;
    params.omnidirectional_calib.cy = 0;
    params.omnidirectional_calib.xc = 1;
    params.omnidirectional_calib.yc = 1;
    params.omnidirectional_calib.c  = 1;
    params.omnidirectional_calib.d  = 1;
    params.omnidirectional_calib.e  = 1;

    params.omnidirectional_calib.length_pol    = 1;
    params.omnidirectional_calib.length_invpol = 1;
    params.omnidirectional_calib.width         = 2;
    params.omnidirectional_calib.height        = 2;

    params.omnidirectional_calib.pol[0]    = 1;
    params.omnidirectional_calib.invpol[0] = 1;
  }

}

} // end of namespace

std::ostream& operator<<(std::ostream& out, const Matcher::parameters& params)
{
  out << "  nms_n                  = " << params.nms_n << std::endl;
  out << "  nms_tau                = " << params.nms_tau << std::endl;
  out << "  match_binsize          = " << params.match_binsize << std::endl;
  out << "  match_radius           = " << params.match_radius << std::endl;
  out << "  match_disp_tolerance   = " << params.match_disp_tolerance << std::endl;
  out << "  outlier_disp_tolerance = " << params.outlier_disp_tolerance << std::endl;
  out << "  outlier_flow_tolerance = " << params.outlier_flow_tolerance << std::endl;
  out << "  multi_stage            = " << params.multi_stage << std::endl;
  out << "  half_resolution        = " << params.half_resolution << std::endl;
  out << "  refinement             = " << params.refinement << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const VisualOdometry::calibration& calibration)
{
  out << "  f  = " << calibration.f << std::endl;
  out << "  cu = " << calibration.cu << std::endl;
  out << "  cv = " << calibration.cv << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const VisualOdometry::omnidirectional_calibration& params)
{
  out << "  poly         = ";
  for (int i = 0; i < params.length_pol; i++) {
    out << params.pol[i] << " ; ";
  }
  out << std::endl;

  out << "  inverse poly = ";
  for (int i = 0; i < params.length_invpol; i++) {
    out << params.invpol[i] << " ; ";
  }
  out << std::endl;

  out << "  xc           = " << params.xc << std::endl;
  out << "  yc           = " << params.yc << std::endl;
  out << "  width        = " << params.width << std::endl;
  out << "  height       = " << params.height << std::endl;
  out << "  fx           = " << params.fx << std::endl;
  out << "  fy           = " << params.fy << std::endl;
  out << "  cx           = " << params.cx << std::endl;
  out << "  cy           = " << params.cy << std::endl;
}

std::ostream& operator<<(std::ostream& out, const VisualOdometry::bucketing& bucketing)
{
  out << "  max_features  = " << bucketing.max_features << std::endl;
  out << "  bucket_width  = " << bucketing.bucket_width << std::endl;
  out << "  bucket_height = " << bucketing.bucket_height << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const VisualOdometry::parameters& params)
{
  out << "Calibration parameters:" << std::endl << params.calib;
  out << "Matcher parameters:" << std::endl << params.match;
  out << "Bucketing parameters:" << std::endl << params.bucket;
  return out;
}

std::ostream& operator<<(std::ostream& out, const VisualOdometryStereo::parameters& params)
{
  out << static_cast<VisualOdometry::parameters>(params);
  out << "Stereo odometry parameters:" << std::endl;
  out << "  base             = " << params.base << std::endl;
  out << "  ransac_iters     = " << params.ransac_iters << std::endl;
  out << "  inlier_threshold = " << params.inlier_threshold << std::endl;
  out << "  reweighting      = " << params.reweighting << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const VisualOdometryMono::parameters& params)
{
  out << static_cast<VisualOdometry::parameters>(params);
  out << "Mono odometry parameters:" << std::endl;
  out << "  camera_height    = " << params.height << std::endl;
  out << "  camera_pitch     = " << params.pitch << std::endl;
  out << "  ransac_iters     = " << params.ransac_iters << std::endl;
  out << "  inlier_threshold = " << params.inlier_threshold << std::endl;
  out << "  motion_threshold = " << params.motion_threshold << std::endl;
  return out;
}


} // end of namespace

