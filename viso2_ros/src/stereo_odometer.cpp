#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/stereo_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <viso_stereo.h>

#include "odometer_base.h"
#include "odometry_params.h"

#include "viso2_ros/VisoInfo.h"
#include "viso2_ros/GetTransform.h"

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


class StereoOdometer : public OdometerBase
{

private:

  boost::shared_ptr<VisualOdometryStereo> visual_odometer_;
  VisualOdometryStereo::parameters visual_odometer_params_;

  // NodeHandles.
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  image_transport::ImageTransport it_;

  // Publishers
  ros::Publisher info_pub_;
  ros::Publisher point_cloud_pub_;
  
  // Client service.
  ros::ServiceClient initial_odom_client_;

  // Subscribers.
  image_transport::SubscriberFilter l_img_sub_;
  image_transport::SubscriberFilter r_img_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> r_info_sub_;

  bool got_lost_;

  // Change reference frame method. 0, 1 or 2. 0 means allways change. 1 and 2 explained below.
  int ref_frame_change_method_;
  bool change_reference_frame_;
  double ref_frame_motion_threshold_; // method 1. Change the reference frame if last motion is small.
  int ref_frame_inlier_threshold_; // method 2. Change the reference frame if the number of inliers is low.
  
  Matrix reference_motion_;

  // Topic sync.
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::CameraInfo> SyncPolicy_;
  typedef message_filters::Synchronizer<SyncPolicy_> Sync_;
  std::shared_ptr<Sync_> sync_;

public:

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  StereoOdometer() : OdometerBase(),
    nh_{}, nhp_{"~"}, it_{nh_}, got_lost_(false), change_reference_frame_(false)
  {
    // Read local parameters.
    odometry_params::loadParams(nhp_, visual_odometer_params_);
    nhp_.param("ref_frame_change_method", ref_frame_change_method_, 0);
    nhp_.param("ref_frame_motion_threshold", ref_frame_motion_threshold_, 5.0);
    nhp_.param("ref_frame_inlier_threshold", ref_frame_inlier_threshold_, 150);

    // Reference motion.
    reference_motion_ = Matrix::eye(4);

    // Publishers.
    info_pub_ = nhp_.advertise<VisoInfo>("info", 1);
    point_cloud_pub_ = nhp_.advertise<PointCloud>("point_cloud", 1);

    // Client service.
    initial_odom_client_ = nhp_.serviceClient<viso2_ros::GetTransform>("get_first_odom");

    // Subscribers.
    l_img_sub_.subscribe(it_, "left_image_rect_color", 5);
    r_img_sub_.subscribe(it_, "right_image_rect_color", 5);
    l_info_sub_.subscribe(nh_, "left_camera_info", 5);
    r_info_sub_.subscribe(nh_, "right_camera_info", 5);

    // Message sync.
    sync_ = std::make_shared<Sync_>(SyncPolicy_(10), l_img_sub_, r_img_sub_, l_info_sub_, r_info_sub_);
    sync_->registerCallback(boost::bind(&StereoOdometer::imageCallback, this, _1, _2, _3, _4));
  }

protected:

  void initOdometer(
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    // Read calibration info from camera info message
    // to fill remaining parameters
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(*l_info_msg, *r_info_msg);
    visual_odometer_params_.base      = model.baseline();
    visual_odometer_params_.calib.cu  = model.left().cx();
    visual_odometer_params_.calib.cv  = model.left().cy();
    visual_odometer_params_.calib.f   = model.left().fx();

    visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
    if (l_info_msg->header.frame_id != "") 
      setSensorFrameId(l_info_msg->header.frame_id);
    ROS_INFO_STREAM("Initialized libviso2 stereo odometry "
                    "with the following parameters:" << std::endl <<
                    visual_odometer_params_ <<
                    "  ref_frame_change_method = " << ref_frame_change_method_ << std::endl <<
                    "  ref_frame_motion_threshold = " << ref_frame_motion_threshold_ << std::endl <<
                    "  ref_frame_inlier_threshold = " << ref_frame_inlier_threshold_);
    setPose(getInitialOdom());
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

    ROS_ASSERT(l_step == r_step);
    ROS_ASSERT(l_image_msg->width == r_image_msg->width);
    ROS_ASSERT(l_image_msg->height == r_image_msg->height);

    int32_t dims[] = {(int32_t)l_image_msg->width, (int32_t)l_image_msg->height, l_step};
    // on first run or when odometer got lost, only feed the odometer with
    // images without retrieving data
    if (first_run || got_lost_)
    {
      visual_odometer_->process(l_image_data, r_image_data, dims);
      got_lost_ = false;
      // on first run publish zero once
      if (first_run)
      {
        tf2::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, l_image_msg->header.stamp);
      }
    }
    else
    {
      bool success = visual_odometer_->process(
          l_image_data, r_image_data, dims, change_reference_frame_);
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

        tf2::Matrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        tf2::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf2::Transform delta_transform(rot_mat, t);

        setPoseCovariance(STANDARD_POSE_COVARIANCE);
        setTwistCovariance(STANDARD_TWIST_COVARIANCE);

        integrateAndPublish(delta_transform, l_image_msg->header.stamp);

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
        tf2::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, l_image_msg->header.stamp);

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
          {
            change_reference_frame_ = false;
            break;
          }
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
      info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
      ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
      info_msg.runtime = time_elapsed.toSec();
      info_pub_.publish(info_msg);
    }
  }

  tf2::Transform getInitialOdom()
  {
    tf2::Transform initial_pose;
    viso2_ros::GetTransform srv;
    if (initial_odom_client_.call(srv))
    {
      tf2::fromMsg(srv.response.tf.transform, initial_pose);
      ROS_WARN_STREAM("[VO->Initial odom:] GetTransform service call successful. The initial odometry is as follows:" << std::endl <<
                      "tx: " << initial_pose.getOrigin().getX() << std::endl << 
                      "ty: " << initial_pose.getOrigin().getY() << std::endl << 
                      "tz: " << initial_pose.getOrigin().getZ() << std::endl << 
                      "qx: " << initial_pose.getRotation().getX() << std::endl << 
                      "qy: " << initial_pose.getRotation().getY() << std::endl << 
                      "qz: " << initial_pose.getRotation().getZ() << std::endl << 
                      "qw: " << initial_pose.getRotation().getW());
    }
    else
    {
      ROS_WARN_STREAM("[VO->Initial odom:] Failed to call GetTransform service. The initial odometry will be assumed as the identity.");
      initial_pose.setIdentity();
    }
    return initial_pose;
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
  // Init ROS.
  ros::init(argc, argv, "stereo_odometer");
  ros::start();

  // Create visual stereo odometer.
  viso2_ros::StereoOdometer odometer;

  ros::spin();

  return 0;
}

