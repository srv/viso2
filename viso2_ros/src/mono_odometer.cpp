#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <viso_mono.h>

#include "odometer_base.h"

namespace viso2_ros
{

class MonoOdometer : public OdometerBase
{

private:

  boost::shared_ptr<VisualOdometryStereo> visual_odometer_;
  VisualOdometryMono::parameters visual_odometer_params_;

  image_transport::CameraSubscriber camera_sub_;

public:

  MonoOdometer(const std::string& transport) : public OdometerBase()
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    odometry_params::loadParams(local_nh, visual_odometer_params_);

    int queue_size;
    local_nh.param("queue_size", queue_size, 1);

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    camera_sub_ = it.subscribeCamera("image", queue_size, &MonoOdometer::imageCallback, transport);
  }

protected:

  void imageCallback(
      const sensor_msgs::ImageConstPtr& image_msg,
      const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
 
    // create odometer if not exists
    if (!visual_odometer_)
    {
      // read calibration info from camera info message
      // to fill remaining odometer parameters
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(info_msg);
      visual_odometer_params_.calib.f  = model.fx();
      visual_odometer_params_.calib.cu = model.cx();
      visual_odometer_params_.calib.cv = model.cy();
      visual_odometer_.reset(new VisualOdometryMono(visual_odometer_params_));
    }

    // convert image if necessary
    uint8_t *image_data;
    int step;
    cv_bridge::CvImageConstPtr cv_ptr;
    if (image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      image_data = const_cast<uint8_t*>(&(image_msg->data[0]));
      step = image_msg->step;
    }
    else
    {
      cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
      image_data = cv_ptr->image.data;
      step = cv_ptr->image.step[0];
    }

    // run the odometer
    int32_t dims[] = {image_msg->width, image_msg->height, step};
    static bool first_run = true;
    if (first_run)
    {
      first_run = false;
      visual_odometer_->process(image_data, dims);
      setSensorFrameId(l_image_msg->header.frame_id);
    }
    else
    {
      if(visual_odometer_->process(image_data, dims))
      {
        Matrix camera_motion = visual_odometer_->getMotion();
        btMatrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);

        btVector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);

        update(delta_transform, image_msg->header.stamp);
      }
      else
      {
        ROS_ERROR("[mono_odometer]: Call to VisualOdometryMono::process() failed!");
      }
    }
  }

};

} // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mono_odometer");
  if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN("mono_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }

  std::string transport = argc > 1 ? argv[1] : "raw";
  viso2_ros::MonoOdometer odometer(transport);
  
  ros::spin();
  return 0;
}

