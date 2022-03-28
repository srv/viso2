#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Range.h>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <viso_stereo.h>

#include <viso2_ros/VisoInfo.h>

#include "stereo_processor.h"
#include "odometer_base.h"
#include "odometry_params.h"

//BMNF
using cv::Mat ;

namespace viso2_ros
{

// some arbitrary values (0.1m^2 linear cov. 10deg^2. angular cov.)
static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0.17, 0, 0,
    0, 0, 0, 0, 0.17, 0,
    0, 0, 0, 0, 0, 0.17 } };
static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
{ { 0.002, 0, 0, 0, 0, 0,
    0, 0.002, 0, 0, 0, 0,
    0, 0, 0.05, 0, 0, 0,
    0, 0, 0, 0.09, 0, 0,
    0, 0, 0, 0, 0.09, 0,
    0, 0, 0, 0, 0, 0.09 } };
static const boost::array<double, 36> BAD_COVARIANCE =
{ { 9999, 0, 0, 0, 0, 0,
    0, 9999, 0, 0, 0, 0,
    0, 0, 9999, 0, 0, 0,
    0, 0, 0, 9999, 0, 0,
    0, 0, 0, 0, 9999, 0,
    0, 0, 0, 0, 0, 9999 } };


class StereoOdometer : public StereoProcessor, public OdometerBase
{

private:

  boost::shared_ptr<VisualOdometryStereo> visual_odometer_;
  VisualOdometryStereo::parameters visual_odometer_params_;

  ros::Publisher point_cloud_pub_;
  ros::Publisher info_pub_;

  //BMNF
  ros::Subscriber altitude_sub_ ;
  float altitude_ = 10.0 ;

  bool got_lost_;

  // change reference frame method. 0, 1 or 2. 0 means allways change. 1 and 2 explained below
  int ref_frame_change_method_;
  bool change_reference_frame_;
  double ref_frame_motion_threshold_; // method 1. Change the reference frame if last motion is small
  int ref_frame_inlier_threshold_; // method 2. Change the reference frame if the number of inliers is low
  Matrix reference_motion_;

  // BMNF: Parameter declaration
  // Version
  int detection_and_tracking_version_ ;
  // Bucketing
  bool enable_bucketing_ ;
  // Feature and descriptor detection
  double contrastThreshold_SIFT_ ; 
  double edgeThreshold_SIFT_ ; 
  double sigma_SIFT_ ;                                
  double hessianThreshold_SURF_ ; 
  int nOctaves_SURF_ ; 
  int nOctaveLayers_ ;
  // Outlier rejection
  int epipolar_constrain_ ;
  double homography_reprojThreshold_ ;
  // Control
  bool constant_altitude_ ;
  double assigned_altitude_ ;

public:

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  StereoOdometer(const std::string& transport) :
    StereoProcessor(transport), OdometerBase(),
    got_lost_(false), change_reference_frame_(false)
  {

    // Read local parameters
    ros::NodeHandle local_nh("~");
    odometry_params::loadParams(local_nh, visual_odometer_params_);

    local_nh.param("ref_frame_change_method", ref_frame_change_method_, 0);
    local_nh.param("ref_frame_motion_threshold", ref_frame_motion_threshold_, 5.0);
    local_nh.param("ref_frame_inlier_threshold", ref_frame_inlier_threshold_, 150);

    // BMNF: Parameter definition
    // Version
    local_nh.param<int>("detection_and_tracking_version", detection_and_tracking_version_, 0) ;
    // Bucketing
    local_nh.param<bool>("enable_bucketing", enable_bucketing_, true) ;
    // Feature and descriptor detection  
    local_nh.param<double>("contrastThreshold_SIFT", contrastThreshold_SIFT_, 0.06) ; 
    local_nh.param<double>("edgeThreshold_SIFT", edgeThreshold_SIFT_, 10.0) ; 
    local_nh.param<double>("sigma_SIFT", sigma_SIFT_, 1.6) ; 
    local_nh.param<double>("hessianThreshold_SURF", hessianThreshold_SURF_, 100.0) ; 
    local_nh.param<int>("nOctaves_SURF", nOctaves_SURF_, 4) ;   
    local_nh.param<int>("nOctaveLayers", nOctaveLayers_, 3) ;
    // Outlier rejection
    local_nh.param<double>("homography_reprojThreshold", homography_reprojThreshold_, 1.0) ; 
    local_nh.param<int>("epipolar_constrain", epipolar_constrain_, 3) ;
    // Control
    local_nh.param<bool>("constant_altitude", constant_altitude_, true) ;
    local_nh.param<double>("assinged_altitude", assigned_altitude_, 3.0) ;

    altitude_sub_ = local_nh.subscribe("/turbot/navigator/altitude_raw", 1, &StereoOdometer::altitudeCB, this) ;

    point_cloud_pub_ = local_nh.advertise<PointCloud>("point_cloud", 1);
    info_pub_ = local_nh.advertise<VisoInfo>("info", 1);

    reference_motion_ = Matrix::eye(4);
    
  }

protected:

  void altitudeCB(const sensor_msgs::RangeConstPtr &msg){

    altitude_ = msg->range ;

  }

  void initOdometer(
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    int queue_size;
    bool approximate_sync;

    ros::NodeHandle local_nh("~");
    local_nh.param<int>("queue_size", queue_size, 10); // 10
    local_nh.param<bool>("approximate_sync", approximate_sync, false);

    // read calibration info from camera info message
    // to fill remaining parameters
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(*l_info_msg, *r_info_msg);
    visual_odometer_params_.base      = model.baseline();
    visual_odometer_params_.calib.cu  = model.left().cx();
    visual_odometer_params_.calib.cv  = model.left().cy();
    visual_odometer_params_.calib.f   = model.left().fx();

    visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
    if (l_info_msg->header.frame_id != "") setSensorFrameId(l_info_msg->header.frame_id);
    ROS_INFO_STREAM("Initialized libviso2 stereo odometry "
                    "with the following parameters:" << visual_odometer_params_ << std::endl <<
                    "  queue_size = " << queue_size << std::endl <<
                    "  approximate_sync = " << approximate_sync << std::endl <<
                    "  ref_frame_change_method = " << ref_frame_change_method_ << std::endl <<
                    "  ref_frame_motion_threshold = " << ref_frame_motion_threshold_ << std::endl <<
                    "  ref_frame_inlier_threshold = " << ref_frame_inlier_threshold_ << std::endl <<
                    "  detection_and_tracking_version = " << detection_and_tracking_version_ << std::endl <<
                    "  enable_bucketing = " << enable_bucketing_ << std::endl <<
                    "  contrastThreshold_SIFT = " << contrastThreshold_SIFT_ << std::endl <<
                    "  edgeThreshold_SIFT = " << edgeThreshold_SIFT_ << std::endl <<
                    "  sigma_SIFT = " << sigma_SIFT_ << std::endl <<
                    "  hessianThreshold_SURF = " << hessianThreshold_SURF_ << std::endl <<
                    "  nOctaves_SURF = " << nOctaves_SURF_ << std::endl <<
                    "  nOctaveLayers = " << nOctaveLayers_ << std::endl <<
                    "  homography_reprojThreshold = " << homography_reprojThreshold_ << std::endl <<
                    "  epipolar_constrain = " << epipolar_constrain_ << std::endl <<
                    "  constant_altitude = " << constant_altitude_ << std::endl <<
                    "  assigned_altitude = " << assigned_altitude_                   
                    );
  }

  void imageCallback(
      const sensor_msgs::ImageConstPtr& l_image_msg,
      const sensor_msgs::ImageConstPtr& r_image_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    ros::WallTime start_time = ros::WallTime::now();
    bool first_run = false;
    // create odometer if not exists
    if (!visual_odometer_)
    {
      first_run = true;
      initOdometer(l_info_msg, r_info_msg);
    }

    // convert images if necessary
    uint8_t *l_image_data, *r_image_data;
    int l_step, r_step;
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
    l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
    l_image_data = l_cv_ptr->image.data;
    l_step = l_cv_ptr->image.step[0];
    r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
    r_image_data = r_cv_ptr->image.data;
    r_step = r_cv_ptr->image.step[0];

    //BMNF: Save image into opencv matrix
    Mat lef_img_new = l_cv_ptr -> image ;
    Mat rig_img_new = r_cv_ptr -> image ;

    ROS_ASSERT(l_step == r_step);
    ROS_ASSERT(l_image_msg->width == r_image_msg->width);
    ROS_ASSERT(l_image_msg->height == r_image_msg->height);

    int32_t dims[] = {l_image_msg->width, l_image_msg->height, l_step};

    // on first run or when odometer got lost, only feed the odometer with
    // images without retrieving data
    if (first_run || got_lost_)
    {
      // BMNF
      if(detection_and_tracking_version_ == 0){

        visual_odometer_->process(l_image_data, r_image_data, dims);

      } else {

        visual_odometer_->new_process(lef_img_new, rig_img_new, change_reference_frame_, enable_bucketing_, detection_and_tracking_version_, nOctaveLayers_,
                                      contrastThreshold_SIFT_, edgeThreshold_SIFT_, sigma_SIFT_, 
                                      hessianThreshold_SURF_, nOctaves_SURF_,
                                      homography_reprojThreshold_, epipolar_constrain_) ; 

      }
      got_lost_ = false;
      // on first run publish zero once
      if (first_run)
      {
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, l_image_msg->header.stamp);
      }
    }
    else
    {
      bool success;

      // BMNF
      if(detection_and_tracking_version_ == 0){

        success = visual_odometer_->process(l_image_data, r_image_data, dims);

      } else {

        success = visual_odometer_->new_process(lef_img_new, rig_img_new, change_reference_frame_, enable_bucketing_, detection_and_tracking_version_, nOctaveLayers_, 
                                                contrastThreshold_SIFT_, edgeThreshold_SIFT_, sigma_SIFT_, 
                                                hessianThreshold_SURF_, nOctaves_SURF_,
                                                homography_reprojThreshold_, epipolar_constrain_) ; // BMNF 03/03/2021, true

      }

      if (success)
      {
        Matrix motion = Matrix::inv(visual_odometer_->getMotion());
        ROS_DEBUG("Found %i matches with %i inliers.",
                  visual_odometer_->getNumberOfMatches(),
                  visual_odometer_->getNumberOfInliers());
        ROS_DEBUG_STREAM("libviso2 returned the following motion:\n" << motion);
        Matrix camera_motion;

        // if image was replaced due to small motion we have to subtract the
        // last motion to get the increment
        if (change_reference_frame_)
        {
          camera_motion = Matrix::inv(reference_motion_) * motion;
        }
        else
        {
          // image was not replaced, report full motion from odometer
          camera_motion = motion;
        }
        reference_motion_ = motion; // store last motion as reference

        tf::Matrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        tf::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);

        setPoseCovariance(STANDARD_POSE_COVARIANCE);
        setTwistCovariance(STANDARD_TWIST_COVARIANCE);

        if((constant_altitude_== true) && (altitude_ < (assigned_altitude_ + 1)) ){

          integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        } else if(constant_altitude_== false) {

          integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        }

        if (point_cloud_pub_.getNumSubscribers() > 0)
        {
          computeAndPublishPointCloud(l_info_msg, l_image_msg, r_info_msg,
                                      visual_odometer_->getMatches(),
                                      visual_odometer_->getInlierIndices());
        }
      }
      else
      {
        setPoseCovariance(BAD_COVARIANCE);
        setTwistCovariance(BAD_COVARIANCE);
        tf::Transform delta_transform;
        delta_transform.setIdentity();

        if((constant_altitude_== true) && (altitude_ < (assigned_altitude_ + 0.5)) ){

          integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        } else if(constant_altitude_== false) {

          integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        }
        // integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        ROS_DEBUG("Call to VisualOdometryStereo::process() failed.");
        ROS_WARN_THROTTLE(10.0, "Visual Odometer got lost!");
        got_lost_ = true;
      }

      if(success)
      {

        // Proceed depending on the reference frame change method
        switch ( ref_frame_change_method_ )
        {
          case 1:
          {
            // calculate current feature flow
            double feature_flow = computeFeatureFlow(visual_odometer_->getMatches());
            change_reference_frame_ = (feature_flow < ref_frame_motion_threshold_);
            ROS_DEBUG_STREAM("Feature flow is " << feature_flow
                << ", marking last motion as "
                << (change_reference_frame_ ? "small." : "normal."));
            break;
          }
          case 2:
          {
            change_reference_frame_ = (visual_odometer_->getNumberOfInliers() > ref_frame_inlier_threshold_);
            break;
          }
          default:
            change_reference_frame_ = false;
        }

      }
      else
        change_reference_frame_ = false;

      if(!change_reference_frame_)
        ROS_DEBUG_STREAM("Changing reference frame");

      // create and publish viso2 info msg
      VisoInfo info_msg;
      info_msg.header.stamp = l_image_msg->header.stamp;
      info_msg.got_lost = !success;
      info_msg.change_reference_frame = !change_reference_frame_;
      info_msg.num_matches = visual_odometer_->getNumberOfMatches();
      // info_msg.num_inliers = visual_odometer_->getNumberOfInliers();

      if(visual_odometer_->getNumberOfMatches() != 0){

        if(visual_odometer_->getNumberOfInliers() <= visual_odometer_->getNumberOfMatches()){

          info_msg.num_inliers = visual_odometer_->getNumberOfInliers();

        } else {

          info_msg.num_inliers = 0 ;

        }

      } else {

        info_msg.num_inliers = 0 ;

      }
      
      ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
      info_msg.runtime = time_elapsed.toSec();
      info_pub_.publish(info_msg);
    }
  }

  double computeFeatureFlow(
      const std::vector<Matcher::p_match>& matches)
  {
    double total_flow = 0.0;
    for (size_t i = 0; i < matches.size(); ++i)
    {
      double x_diff = matches[i].u1c - matches[i].u1p;
      double y_diff = matches[i].v1c - matches[i].v1p;
      total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
    }
    return total_flow / matches.size();
  }

  void computeAndPublishPointCloud(
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::ImageConstPtr& l_image_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg,
      const std::vector<Matcher::p_match>& matches,
      const std::vector<int32_t>& inlier_indices)
  {
    try
    {
      cv_bridge::CvImageConstPtr cv_ptr;
      cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
      // read calibration info from camera info message
      image_geometry::StereoCameraModel model;
      model.fromCameraInfo(*l_info_msg, *r_info_msg);
      PointCloud::Ptr point_cloud(new PointCloud());
      point_cloud->header.frame_id = getSensorFrameId();
      point_cloud->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
      point_cloud->width = 1;
      point_cloud->height = inlier_indices.size();
      point_cloud->points.resize(inlier_indices.size());

      for (size_t i = 0; i < inlier_indices.size(); ++i)
      {
        const Matcher::p_match& match = matches[inlier_indices[i]];
        cv::Point2d left_uv;
        left_uv.x = match.u1c;
        left_uv.y = match.v1c;
        cv::Point3d point;
        double disparity = match.u1c - match.u2c;
        model.projectDisparityTo3d(left_uv, disparity, point);
        point_cloud->points[i].x = point.x;
        point_cloud->points[i].y = point.y;
        point_cloud->points[i].z = point.z;
        cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(left_uv.y,left_uv.x);
        point_cloud->points[i].r = color[0];
        point_cloud->points[i].g = color[1];
        point_cloud->points[i].b = color[2];
      }
      ROS_DEBUG("Publishing point cloud with %zu points.", point_cloud->size());
      point_cloud_pub_.publish(point_cloud);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
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

