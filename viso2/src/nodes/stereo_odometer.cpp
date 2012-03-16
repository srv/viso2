
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include "viso_stereo.h"

void increment(int* value)
{
  ++(*value);
}

class StereoOdometer
{

private:

  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  int queue_size_;
  
  ros::WallTimer check_synced_timer_;
  int left_received_, right_received_, left_info_received_, right_info_received_, all_received_;

public:

  StereoOdometer(const std::string& transport) :
    left_received_(0), right_received_(0), left_info_received_(0), right_info_received_(0), all_received_(0)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    
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
  }

protected:

  void imageCb(const sensor_msgs::ImageConstPtr& l_image_msg,
               const sensor_msgs::ImageConstPtr& r_image_msg,
               const sensor_msgs::CameraInfoConstPtr& l_info_msg,
               const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
 
    ++all_received_; // For error checking

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
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

