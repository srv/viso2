#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <viso_stereo.h>

#include "stereo_processor.h"
#include "odometer_base.h"
#include "odometry_params.h"

namespace viso2_ros
{

class StereoOdometer : public StereoProcessor, public OdometerBase
{

private:

  boost::shared_ptr<VisualOdometryStereo> visual_odometer_;
  VisualOdometryStereo::parameters visual_odometer_params_;

  bool replace_;


public:

  StereoOdometer(const std::string& transport) : StereoProcessor(transport), OdometerBase(), replace_(false)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    odometry_params::loadParams(local_nh, visual_odometer_params_);
  }

protected:

  void imageCallback(
      const sensor_msgs::ImageConstPtr& l_image_msg,
      const sensor_msgs::ImageConstPtr& r_image_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
 
    bool first_run = false;
    // create odometer if not exists
    if (!visual_odometer_)
    {
      first_run = true;
      // read calibration info from camera info message
      // to fill remaining parameters
      image_geometry::StereoCameraModel model;
      model.fromCameraInfo(*l_info_msg, *r_info_msg);
      visual_odometer_params_.base = model.baseline();
      visual_odometer_params_.calib.f = model.left().fx();
      visual_odometer_params_.calib.cu = model.left().cx();
      visual_odometer_params_.calib.cv = model.left().cy();
      visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
      setSensorFrameId(l_info_msg->header.frame_id);
      ROS_INFO_STREAM("[stereo_odometer]: Initialized libviso2 stereo odometry "
                      "with the following parameters:" << std::endl << 
                      visual_odometer_params_);
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
    // on first run, only feed the odometer with first image pair without
    // retrieving data
    if (first_run)
    {
      visual_odometer_->process(l_image_data, r_image_data, dims);
    }
    else
    {
      if(visual_odometer_->process(l_image_data, r_image_data, dims, replace_))
      {
        replace_ = false;
        Matrix camera_motion = visual_odometer_->getMotion();
        btMatrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);

        ROS_DEBUG_STREAM("[stereo_odometer]: libviso2 returned the following motion:\n" << camera_motion);

        btVector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);

        integrateAndPublish(delta_transform, l_image_msg->header.stamp);
      }
      else
      {
        ROS_DEBUG("[stereo_odometer]: Call to VisualOdometryStereo::process() failed. Assuming motion too small.");
        replace_ = true;
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
             "\t$ rosrun viso2_ros stereo_odometer stereo:=narrow_stereo image:=image_rect");
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

