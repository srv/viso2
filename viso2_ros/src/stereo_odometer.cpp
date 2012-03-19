
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include "viso_stereo.h"

// for sync checking
void increment(int* value)
{
  ++(*value);
}

class StereoOdometer
{

private:

  // publisher and subscriber
  ros::Publisher odom_pub_;
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  int queue_size_;

  // for sync checking
  ros::WallTimer check_synced_timer_;
  int left_received_, right_received_, left_info_received_, right_info_received_, all_received_;

  // tf related
  std::string frame_id_;
  std::string child_frame_id_;
  bool publish_tf_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  boost::shared_ptr<VisualOdometryStereo> visual_odometer_;
  VisualOdometryStereo::parameters visual_odometer_params_;

  // the current integrated camera pose
  tf::Transform integrated_pose_;
  // timestamp of the last update
  ros::Time last_update_time_;

public:

  StereoOdometer(const std::string& transport) :
    left_received_(0), right_received_(0), left_info_received_(0), right_info_received_(0), all_received_(0)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    loadParams(local_nh, visual_odometer_params_);

    local_nh.param("frame_id", frame_id_, std::string("visual_odom"));
    local_nh.param("child_frame_id", child_frame_id_, std::string("base_link"));
    local_nh.param("publish_tf", publish_tf_, true);
    
    // Resolve topic names
    ros::NodeHandle nh;
    std::string stereo_ns = nh.resolveName("stereo");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image"));
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image"));

    std::string left_info_topic = stereo_ns + "/left/camera_info";
    std::string right_info_topic = stereo_ns + "/right/camera_info";

    ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s\n\t* %s", 
        left_topic.c_str(), right_topic.c_str(),
        left_info_topic.c_str(), right_info_topic.c_str());

    // Subscribe to four input topics.
    image_transport::ImageTransport it(nh);
    left_sub_.subscribe(it, left_topic, 1, transport);
    right_sub_.subscribe(it, right_topic, 1, transport);
    left_info_sub_.subscribe(nh, left_info_topic, 1);
    right_info_sub_.subscribe(nh, right_info_topic, 1);

    // Complain every 15s if the topics appear unsynchronized
    left_sub_.registerCallback(boost::bind(increment, &left_received_));
    right_sub_.registerCallback(boost::bind(increment, &right_received_));
    left_info_sub_.registerCallback(boost::bind(increment, &left_info_received_));
    right_info_sub_.registerCallback(boost::bind(increment, &right_info_received_));
    check_synced_timer_ = nh.createWallTimer(ros::WallDuration(15.0),
                                             boost::bind(&StereoOdometer::checkInputsSynchronized, this));

    // Synchronize input topics. Optionally do approximate synchronization.
    local_nh.param("queue_size", queue_size_, 5);
    bool approx;
    local_nh.param("approximate_sync", approx, false);
    if (approx)
    {
      approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size_),
                                                   left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
      approximate_sync_->registerCallback(boost::bind(&StereoOdometer::imageCb, this, _1, _2, _3, _4));
    }
    else
    {
      exact_sync_.reset( new ExactSync(ExactPolicy(queue_size_),
                                       left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
      exact_sync_->registerCallback(boost::bind(&StereoOdometer::imageCb, this, _1, _2, _3, _4));
    }

    // advertise
    odom_pub_ = local_nh.advertise<nav_msgs::Odometry>("odometry", 1);

    integrated_pose_.setIdentity();
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

  void imageCb(const sensor_msgs::ImageConstPtr& l_image_msg,
               const sensor_msgs::ImageConstPtr& r_image_msg,
               const sensor_msgs::CameraInfoConstPtr& l_info_msg,
               const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
 
    ++all_received_; // For error checking

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
    uint8_t l_step, r_step;
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
      last_update_time_ = l_image_msg->header.stamp;
      visual_odometer_->process(l_image_data, r_image_data, dims);
    }
    else
    {
      if(visual_odometer_->process(l_image_data, r_image_data, dims))
      {
        Matrix camera_motion = Matrix::inv(visual_odometer_->getMotion());
        btMatrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        btVector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);
        integrated_pose_ *= delta_transform;

        // transform integrated pose to base link
        tf::StampedTransform child_to_camera;
        try
        {
          tf_listener_.lookupTransform(
              child_frame_id_,
              l_image_msg->header.frame_id,
              ros::Time(0), child_to_camera);
        }
        catch (tf::TransformException& ex)
        {
          ROS_WARN("TransformException: %s", ex.what());
          ROS_WARN("The tf from '%s' to '%s' does not seem to be available, "
                   "will assume it as identity!", 
                   child_frame_id_.c_str(),
                   l_image_msg->header.frame_id.c_str());
          child_to_camera.setIdentity();
        }

        tf::Transform child_transform = child_to_camera * integrated_pose_ * child_to_camera.inverse();

        // calculate twist
        tf::Transform delta_child_transform = child_to_camera * delta_transform * child_to_camera.inverse();
        ros::Time current_time = l_image_msg->header.stamp;
        double delta_t = (current_time - last_update_time_).toSec();
        last_update_time_ = current_time;

        nav_msgs::Odometry odometry;
        odometry.header.stamp = l_image_msg->header.stamp;
        odometry.header.frame_id = frame_id_;
        odometry.child_frame_id = child_frame_id_;
        tf::poseTFToMsg(child_transform, odometry.pose.pose);

        // TODO fill covariances
        odom_pub_.publish(odometry);

        if (publish_tf_)
        {
          tf_broadcaster_.sendTransform(
              tf::StampedTransform(child_transform, l_image_msg->header.stamp,
              frame_id_, child_frame_id_));
        }
      }
      else
      {
        ROS_ERROR("[stereo_odometer]: Call to VisualOdometryStereo::process() failed!");
      }
    }
  }

  void checkInputsSynchronized()
  {
    int threshold = 3 * all_received_;
    if (left_received_ >= threshold || right_received_ >= threshold || 
        left_info_received_ >= threshold || right_info_received_ >= threshold) {
      ROS_WARN("[stereo_odometer] Low number of synchronized left/right/left_info/right_info tuples received.\n"
               "Left images received:       %d (topic '%s')\n"
               "Right images received:      %d (topic '%s')\n"
               "Left camera info received:  %d (topic '%s')\n"
               "Right camera info received: %d (topic '%s')\n"
               "Synchronized tuples: %d\n"
               "Possible issues:\n"
               "\t* stereo_image_proc is not running.\n"
               "\t  Does `rosnode info %s` show any connections?\n"
               "\t* The cameras are not synchronized.\n"
               "\t  Try restarting stereo_odometer with parameter _approximate_sync:=True\n"
               "\t* The network is too slow. One or more images are dropped from each tuple.\n"
               "\t  Try restarting stereo_odometer, increasing parameter 'queue_size' (currently %d)",
               left_received_, left_sub_.getTopic().c_str(),
               right_received_, right_sub_.getTopic().c_str(),
               left_info_received_, left_info_sub_.getTopic().c_str(),
               right_info_received_, right_info_sub_.getTopic().c_str(),
               all_received_, ros::this_node::getName().c_str(), queue_size_);
    }
  }
};


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
  StereoOdometer odometer(transport);
  
  ros::spin();
  return 0;
}

