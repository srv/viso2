#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Range.h>

#include <viso_mono.h>

#include <viso2_ros/VisoInfo.h>

#include "odometer_base.h"
#include "odometry_params.h" // charge the NodeHandle params comming from launch file

namespace viso2_ros
{

class MonoOdometer : public OdometerBase
{

/* BMNF: Variables declaration */
private:

  boost::shared_ptr<VisualOdometryMono> visual_odometer_;
  VisualOdometryMono::parameters visual_odometer_params_;

  image_transport::CameraSubscriber camera_sub_;
  ros::Subscriber altitude_sub; // subscriber to the altitude message

  ros::Publisher info_pub_;

  bool replace_;
  double cameraHeight;

public:

  /* BMNF: First calls OdometerBase () from odometer_base.h and then initializes the node, 
  loads the local parameters, subscribes to the relevant altitude and camera topics.
  Finally publish a message VisoInfo type. */
  MonoOdometer(const std::string& transport) : OdometerBase(), replace_(false)
  {

    /* BMNF: Read & load local parameters. */
    ros::NodeHandle local_nh("~");
    odometry_params::loadParams(local_nh, visual_odometer_params_);

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    /* BMNF: Subscriber to image topic. */
    /* BMNF: Per que la coa es d'1? */
    camera_sub_ = it.subscribeCamera("image", 1, &MonoOdometer::imageCallback, this, transport);

    /* BMNF: Subscriber to altitude topic. */
    /* BMNF: Per que la coa es d'2? */
    altitude_sub = nh.subscribe("altitude", 2, &MonoOdometer::altitudeCallback, this);

    info_pub_ = local_nh.advertise<VisoInfo>("info", 1);

  }

protected:

  /* BMNF: When the program receives an altitude topic, it enters to this callback and prints the value of the
  range of the altitude message. */
  void altitudeCallback (const sensor_msgs::RangeConstPtr& altitudemsg)
    {
        cameraHeight= altitudemsg->range;
        ROS_INFO_STREAM("camera height " << cameraHeight);

    }

  /* BMNF: When the program receives an image topic it enters to this callback. */
  void imageCallback(
      const sensor_msgs::ImageConstPtr& image_msg,
      const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    ros::WallTime start_time = ros::WallTime::now();
 
    bool first_run = false;

    /* Create odometer if not exists. */
    if (!visual_odometer_)
    {
      first_run = true;

      /* BMNF: Read calibration information from camera info and obtain the focal point and the principals points of 
      the camera to fill in the remaining odometer parameters. In addition it also gets the frame id of the camera. */
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(info_msg);
      visual_odometer_params_.calib.f  = model.fx();
      visual_odometer_params_.calib.cu = model.cx();
      visual_odometer_params_.calib.cv = model.cy();
      visual_odometer_.reset(new VisualOdometryMono(visual_odometer_params_));
      if (image_msg->header.frame_id != "") setSensorFrameId(image_msg->header.frame_id); 
      ROS_INFO_STREAM("Initialized libviso2 mono odometry "
                      "with the following parameters:" << std::endl << 
                      visual_odometer_params_);
    }

    /* BMNF: Convert image if necessary. If the image is encoded in MONO8 the program obtain 
    the pointer to rectified image. If the image is not in MONO8 the program transforms 
    it to this encoding and then obtains the pointer to rectified image. */
    uint8_t *image_data;
    int step; 
    cv_bridge::CvImageConstPtr cv_ptr; 
    if (image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      image_data = const_cast<uint8_t*>(&(image_msg->data[0]));
      step = image_msg->step; // BMNF:???
    }
    else
    {
      cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8); 
      image_data = cv_ptr->image.data;
      step = cv_ptr->image.step[0]; // BMNF:???
    }

    /* BMNF: Run the odometer. On first run, only feed the odometer with first image pair without retrieving data.
       If it is not the first iteration there are two possibilities. That the program is able to process 
       the image or not. If the program is able to process it, the program can calculate the translation and rotation 
       matrix to obtain the distance between the camera and the image. Otherwise it only computes an identity matrix.
       In both cases the information is published.
     */
    int32_t dims[] = {image_msg->width, image_msg->height, step};
  
    if (first_run)
    { // fbf 22/07/2020 pass the cameraHeight from the continuously obtained topic given by the altitude estimator 
      visual_odometer_->process(image_data, dims,cameraHeight,true); //cameraHeigh will update this value at every odometry calculation
      tf::Transform delta_transform;
      delta_transform.setIdentity();
      integrateAndPublish(delta_transform, image_msg->header.stamp);
    }
    else
    {
      if(replace_){
        std::cout << "Replace is TRUE" << std::endl;
      }else{
        std::cout << "Replace is FALSE" << std::endl;
      }
      bool success = visual_odometer_->process(image_data, dims, replace_, cameraHeight, true); // BMNF: Aquiiiiiiiiiiiiiiiiiiiiiii!!!!
      if(success)
      {
        ROS_INFO("SUCCESS");
        replace_ = false;
        Matrix camera_motion = Matrix::inv(visual_odometer_->getMotion());
        ROS_DEBUG("Found %i matches with %i inliers.", 
                  visual_odometer_->getNumberOfMatches(),
                  visual_odometer_->getNumberOfInliers());
        ROS_DEBUG_STREAM("libviso2 returned the following motion:\n" << camera_motion);

        tf::Matrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        tf::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);

        integrateAndPublish(delta_transform, image_msg->header.stamp);
      }
      else
      {
        ROS_INFO("Call to VisualOdometryMono::process() failed. Assuming motion too small.");
        replace_ = true;
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, image_msg->header.stamp);
      }

      /* BMNF: Create and publish viso2 info msg. Topic: /mono_odometer/info. */
      VisoInfo info_msg;
      info_msg.header.stamp = image_msg->header.stamp;
      info_msg.got_lost = !success;
      info_msg.change_reference_frame = false;
      info_msg.num_matches = visual_odometer_->getNumberOfMatches();
      info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
      ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
      info_msg.runtime = time_elapsed.toSec();
      info_pub_.publish(info_msg);
    }
  }
};

} // end of namespace


int main(int argc, char **argv)
{
  
  /* Node initialization */
  ros::init(argc, argv, "mono_odometer");

  /* Search the topic of rectified images */
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

