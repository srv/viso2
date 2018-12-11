#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <viso_mono.h>

#include <viso2_ros/VisoInfo.h>

// author: André Aguiar
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <Eigen/Dense>
#include <iostream>
#include <kalman_filter.h>

#include <viso2_ros/XYZ.h>
#include <viso2_ros/angVel.h>
#include <viso2_ros/rot.h>

/* KF definitions */

const double b_x0 = 0.25;
const double b_y0 = 0.25;
const double P_xx = 0.25;

const double rot_threshold = 0.2;

// ---- //

#include "odometer_base.h"
#include "odometry_params.h"

namespace viso2_ros {

using namespace std;

class MonoOdometer : public OdometerBase {

private:
  boost::shared_ptr<VisualOdometryMono> visual_odometer_;
  VisualOdometryMono::parameters        visual_odometer_params_;

  image_transport::CameraSubscriber camera_sub_;

  ros::Publisher info_pub_;

  bool replace_;

  // author: André Aguiar
  ros::Time                  previous_time;
  image_transport::Publisher img_pub;
  ros::Publisher gyro_w_pub, kf_w_pub, vo_w_pub, acc_rot_pub,
      kf_rot_pub, vo_rot_pub, bias_pub;
  tf::Vector3      trans;
  sensor_msgs::Imu imu_data;

  cv::Mat left, right;

  message_filters::Subscriber<sensor_msgs::Image>      cam_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
  message_filters::Subscriber<sensor_msgs::Imu>        imu_sub;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Imu>
                                                      MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync>                             sync_;

  KalmanFilter*   kf;
  Eigen::MatrixXd A, B, H, R, Q, P;
  Eigen::VectorXd X;
  Eigen::VectorXd U;
  Eigen::VectorXd Z;

  viso2_ros::angVel g_w, kf_w, vo_w;
  viso2_ros::rot    acc_rot, kf_rot, vo_rot;
  viso2_ros::XYZ    bias;
  int               n, m, l;

  bool rotating;
  // ---- //

public:
  MonoOdometer(const std::string& transport)
      : OdometerBase(), replace_(false)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    odometry_params::loadParams(local_nh, visual_odometer_params_);

    ros::NodeHandle                 nh;
    image_transport::ImageTransport it(nh);

    // Timestamp synchronization using message_filters/ApproximateTime
    // Callback funtion has 3 inputs (image, camera_info, imu_data)
    // author: André Aguiar
    cam_sub.subscribe(nh, "/camera_out/image_rect", 1);
    info_sub.subscribe(nh, "camera_out/camera_info", 1);
    imu_sub.subscribe(nh, "/mavros/imu/data", 1000);
    sync_.reset(
        new Sync(MySyncPolicy(1000), cam_sub, info_sub, imu_sub));
    sync_->registerCallback(
        boost::bind(&MonoOdometer::callback, this, _1, _2, _3));

    info_pub_   = local_nh.advertise<VisoInfo>("info", 1);
    kf_w_pub    = local_nh.advertise<angVel>("kf_w", 1);
    gyro_w_pub  = local_nh.advertise<angVel>("gyro_w", 1);
    vo_w_pub    = local_nh.advertise<angVel>("viso_w", 1);
    acc_rot_pub = local_nh.advertise<rot>("acc_angle", 1);
    kf_rot_pub  = local_nh.advertise<rot>("kf_angle", 1);
    vo_rot_pub  = local_nh.advertise<rot>("vo_angle", 1);
    bias_pub    = local_nh.advertise<XYZ>("bias", 1);

#ifdef DEBUG
    img_pub = it.advertise("matches_img", 1);
#endif

    /* KALMAN FILTER INITIALIZATION */
    rotating = false;

    n = 5;
    m = 2;
    l = 3;

    g_w.wx     = 0.0;
    g_w.wy     = 0.0;
    kf_w.wx    = 0.0;
    kf_w.wy    = 0.0;
    vo_w.wx    = 0.0;
    vo_w.wy    = 0.0;
    acc_rot.Rx = 0.0;
    kf_rot.Rx  = 0.0;
    vo_rot.Rx  = 0.0;
    bias.x     = 0.0;
    bias.y     = 0.0;

    A = Eigen::MatrixXd(n, n);
    B = Eigen::MatrixXd(n, m);
    H = Eigen::MatrixXd(l, n);

    U = Eigen::MatrixXd(m, 1);
    Z = Eigen::MatrixXd(l, 1);

    X = Eigen::VectorXd(n, 1);
    X << 0, b_x0, b_y0, 0, 0;

    P = Eigen::MatrixXd(n, n);
    P << P_xx, 0, 0, 0, 0, 0, P_xx, 0, 0, 0, 0, 0, P_xx, 0, 0, 0, 0,
        0, P_xx, 0, 0, 0, 0, 0, P_xx;

    kf = new KalmanFilter(X, P);
    /* ----- */
  }

protected:
  // function that draws the matches between current and previous
  // frames &
  // publishes it
  // author:  André Aguiar
  void drawMatches(std::vector<Matcher::p_match> matches)
  {
    std::vector<cv::DMatch>   m;
    std::vector<cv::KeyPoint> keyPointRight;
    std::vector<cv::KeyPoint> keyPointLeft;
    std::vector<cv::Point2f>  auxRight;
    std::vector<cv::Point2f>  auxLeft;

    for (size_t i = 0; i < matches.size(); i++) {
      auxRight.push_back(cv::Point2f(matches[i].u1c, matches[i].v1c));
      auxLeft.push_back(cv::Point2f(matches[i].u1p, matches[i].v1p));
    }
    cv::KeyPoint::convert(auxRight, keyPointRight);
    cv::KeyPoint::convert(auxLeft, keyPointLeft);

    for (size_t j = 0; j < auxRight.size(); j++)
      m.push_back(cv::DMatch(j, j, 0));

    cv::Mat img_matches;
    cv::drawMatches(left, keyPointLeft, right, keyPointRight, m,
                    img_matches);

    sensor_msgs::ImagePtr img =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_matches)
            .toImageMsg();
    img_pub.publish(img);
  }

  // function called in before each iteration of kalman filter to
  // contruct the
  // models
  // author: André Aguiar
  void contructModel(double dt)
  {
    A << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0;

    B << dt, 0, 0, 0, 0, 0, 1, 0, 0, 1;

    if (rotating == false)
      H << 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1;
    else {
      H << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
      (*kf).resetP();
    }
  }

  // Callback funtion that receives (image, camera_info, gyro_data)
  //                  that calculates VO rotation and translation
  //                  that integrates gyro measurements and calculates
  //                  rotation matrix
  //                  that allows the system to run with grayscale and
  //                  hue
  //                  image color channel
  // edited by:       André Aguiar
  void callback(const sensor_msgs::ImageConstPtr&      image_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg,
                const sensor_msgs::ImuConstPtr&        imu_msg)
  {

    ros::WallTime start_time = ros::WallTime::now();

    bool first_run = false;
    // create odometer if not exists
    if (!visual_odometer_) {
      first_run = true;
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(info_msg);
      visual_odometer_params_.calib.f  = model.fx();
      visual_odometer_params_.calib.cu = model.cx();
      visual_odometer_params_.calib.cv = model.cy();
      visual_odometer_.reset(
          new VisualOdometryMono(visual_odometer_params_));
      if (image_msg->header.frame_id != "")
        setSensorFrameId(image_msg->header.frame_id);
      ROS_INFO_STREAM("Initialized libviso2 mono odometry "
                      "with the following parameters:"
                      << std::endl
                      << visual_odometer_params_);
    }

    uint8_t*                   image_data;
    int                        step;
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
        image_msg, sensor_msgs::image_encodings::MONO8);
    if (image_msg->encoding == sensor_msgs::image_encodings::MONO8) {
      image_data = const_cast<uint8_t*>(&(image_msg->data[0]));
      step       = image_msg->step;
    } else {
      image_data = cv_ptr->image.data;
      step       = cv_ptr->image.step[0];
    }

    // run the inertial-odometer
    int32_t dims[] = {image_msg->width, image_msg->height, step};
    if (first_run) {
      visual_odometer_->process(image_data, dims);
      tf::Transform delta_transform;
      delta_transform.setIdentity();
      // integrateAndPublish(delta_transform, image_msg->header.stamp,
      // other);
      previous_time = ros::Time::now();

#ifdef DEBUG
      left = cv::Mat(cv::Mat::zeros(cv_ptr->image.rows,
                                    cv_ptr->image.cols,
                                    cv_ptr->image.type()));
      right = cv::Mat(cv::Mat::zeros(cv_ptr->image.rows,
                                     cv_ptr->image.cols,
                                     cv_ptr->image.type()));
#endif
    } else {
      bool success = visual_odometer_->process(image_data, dims);
      if (success) {
        replace_ = false;
        Matrix camera_motion =
            Matrix::inv(visual_odometer_->getMotion());
        ROS_DEBUG("Found %i matches with %i inliers.",
                  visual_odometer_->getNumberOfMatches(),
                  visual_odometer_->getNumberOfInliers());
        ROS_DEBUG_STREAM("libviso2 returned the following motion:\n"
                         << camera_motion);

        /* visual odometry output */
        tf::Matrix3x3 rot_mat(
            camera_motion.val[0][0], camera_motion.val[0][1],
            camera_motion.val[0][2], camera_motion.val[1][0],
            camera_motion.val[1][1], camera_motion.val[1][2],
            camera_motion.val[2][0], camera_motion.val[2][1],
            camera_motion.val[2][2]);
        tf::Vector3 t(camera_motion.val[0][3],
                      camera_motion.val[1][3],
                      camera_motion.val[2][3]);

        double roll, pitch, yaw;
        rot_mat.getRPY(roll, pitch, yaw);
        /* ********************* */

        /*     kalman filter     */
        double dt     = (ros::Time::now() - previous_time).toSec();
        previous_time = ros::Time::now();

        double acc_roll = atan2(imu_msg->linear_acceleration.y,
                                imu_msg->linear_acceleration.z);
        double w_x = roll / dt;
        double w_y = pitch / dt;

        U << w_x, w_y;
        Z << acc_roll, imu_msg->angular_velocity.x,
            -imu_msg->angular_velocity.z;

        rotating = (abs(Z[2]) > rot_threshold);
        contructModel(dt);
        (*kf).system(A, B, H);
        if (!(*kf).process(Z, U, dt)) {
          Eigen::VectorXd state = (*kf).getState();

          /* PLOT DATA */

          g_w.wx     = imu_msg->angular_velocity.x;
          g_w.wy     = -imu_msg->angular_velocity.z;
          kf_w.wx    = state[3];
          kf_w.wy    = state[4];
          vo_w.wx    = w_x;
          vo_w.wy    = w_y;
          acc_rot.Rx = acc_roll;
          kf_rot.Rx  = state[0];
          vo_rot.Rx  = vo_rot.Rx;
          bias.x     = state[1];
          bias.y     = state[2];

          gyro_w_pub.publish(g_w);
          kf_w_pub.publish(kf_w);
          vo_w_pub.publish(vo_w);
          acc_rot_pub.publish(acc_rot);
          kf_rot_pub.publish(kf_rot);
          vo_rot_pub.publish(vo_rot);
          bias_pub.publish(bias);
        } else
          ROS_ERROR("Invalid matrix dimensions\n");
#ifdef DEBUG
        std::vector<Matcher::p_match> matches =
            visual_odometer_->getMatches();
        right = cv::Mat(cv_ptr->image);
        drawMatches(matches);
        left = cv::Mat(cv_ptr->image);
#endif
      }
      if (!success) {
        gyro_w_pub.publish(g_w);
        kf_w_pub.publish(kf_w);
        vo_w_pub.publish(vo_w);
        acc_rot_pub.publish(acc_rot);
        kf_rot_pub.publish(kf_rot);
        vo_rot_pub.publish(vo_rot);
        bias_pub.publish(bias);
      }

      // create and publish viso2 info msg
      VisoInfo info_msg;
      info_msg.header.stamp           = image_msg->header.stamp;
      info_msg.got_lost               = !success;
      info_msg.change_reference_frame = false;
      info_msg.num_matches = visual_odometer_->getNumberOfMatches();
      info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
      ros::WallDuration time_elapsed =
          ros::WallTime::now() - start_time;
      info_msg.runtime = time_elapsed.toSec();
      info_pub_.publish(info_msg);
    }
  }
};

} // end of namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mono_odometer");
  if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN(
        "mono_odometer needs rectified input images. The used image "
        "topic is '%s'. Are you sure the images are rectified?",
        ros::names::remap("image").c_str());
  }

  std::string             transport = argc > 1 ? argv[1] : "raw";
  viso2_ros::MonoOdometer odometer(transport);

  ros::spin();
  return 0;
}

