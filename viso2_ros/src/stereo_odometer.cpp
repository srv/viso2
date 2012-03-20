
#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>

#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include "stereo_processor.h"
#include "odometer_base.h"

#include "viso_stereo.h"

namespace viso2_ros
{

class StereoOdometer : public StereoProcessor, public OdometerBase
{

private:

  boost::shared_ptr<VisualOdometryStereo> visual_odometer_;
  VisualOdometryStereo::parameters visual_odometer_params_;

public:

  StereoOdometer(const std::string& transport) : StereoProcessor(transport), OdometerBase()
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    loadParams(local_nh, visual_odometer_params_);
  }

protected:

  void loadParams(const ros::NodeHandle& local_nh, VisualOdometryStereo::parameters& params)
  {
    local_nh.getParam("ransac_iters",     params.ransac_iters);
    local_nh.getParam("inlier_threshold", params.inlier_threshold);
    local_nh.getParam("reweighting",      params.reweighting);
    loadGeneralParams(local_nh, params);
  }

  void loadGeneralParams(const ros::NodeHandle& local_nh, VisualOdometry::parameters& params)
  {
    loadMatcherParams(local_nh, params.match);
    loadBucketingParams(local_nh, params.bucket);
  }

  void loadMatcherParams(const ros::NodeHandle& local_nh, Matcher::parameters& params)
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

  void loadBucketingParams(const ros::NodeHandle& local_nh, VisualOdometry::bucketing& bucketing)
  {
    local_nh.getParam("max_features",  bucketing.max_features);
    local_nh.getParam("bucket_width",  bucketing.bucket_width);
    local_nh.getParam("bucket_height", bucketing.bucket_height);
  }

  void imageCallback(
      const sensor_msgs::ImageConstPtr& l_image_msg,
      const sensor_msgs::ImageConstPtr& r_image_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
 
    // create odometer if not exists
    if (!visual_odometer_)
    {
      // read calibration info from camera info message
      image_geometry::StereoCameraModel model;
      model.fromCameraInfo(*l_info_msg, *r_info_msg);
      visual_odometer_params_.base = model.baseline();
      visual_odometer_params_.calib.f = model.left().fx();
      visual_odometer_params_.calib.cu = model.left().cx();
      visual_odometer_params_.calib.cv = model.left().cy();
      visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
    }

    // convert images if necessary
    uint8_t *l_image_data, *r_image_data;
    int l_step, r_step;
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
    if (l_image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      l_image_data = const_cast<uint8_t*>(&(l_image_msg->data[0]));
      l_step = l_image_msg->step;
    }
    else
    {
      l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
      l_image_data = l_cv_ptr->image.data;
      l_step = l_cv_ptr->image.step[0];
    }
    if (r_image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      r_image_data = const_cast<uint8_t*>(&(r_image_msg->data[0]));
      r_step = r_image_msg->step;
    }
    else
    {
      r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
      r_image_data = r_cv_ptr->image.data;
      r_step = r_cv_ptr->image.step[0];
    }

    ROS_ASSERT(l_step == r_step);
    ROS_ASSERT(l_image_msg->width == r_image_msg->width);
    ROS_ASSERT(l_image_msg->height == r_image_msg->height);

    // run the odometer
    int32_t dims[] = {l_image_msg->width, l_image_msg->height, l_step};
    static bool first_run = true;
    if (first_run)
    {
      first_run = false;
      visual_odometer_->process(l_image_data, r_image_data, dims);
      setSensorFrameId(l_image_msg->header.frame_id);
    }
    else
    {
      if(visual_odometer_->process(l_image_data, r_image_data, dims))
      {
        Matrix camera_motion = visual_odometer_->getMotion();
        btMatrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);

        btVector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);

        update(delta_transform, l_image_msg->header.stamp);
      }
      else
      {
        ROS_ERROR("[stereo_odometer]: Call to VisualOdometryStereo::process() failed!");
      }
    }
  }

};

} // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_odometer");
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun viso2 stereo_odometer stereo:=narrow_stereo image:=image_rect");
  }
  if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN("stereo_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }

  std::string transport = argc > 1 ? argv[1] : "raw";
  viso2_ros::StereoOdometer odometer(transport);
  
  ros::spin();
  return 0;
}

