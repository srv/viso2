/*
 *  Created on: 2020-2021
 *  Author: Bo Miquel Nordfeldt Fiol
 *  Name: testSIFT
 */

/* Includes. */
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>  
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/calib3d.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>

using std::vector ;

using cv::Mat ;
using cv::Ptr ;
using cv::KeyPoint ;

using cv::ORB ;
using cv::xfeatures2d::SIFT ;

using namespace sensor_msgs;
using namespace message_filters;    

vector<KeyPoint> left_previous_kpts, right_previous_kpts ;
Mat left_img_previous ;
Mat left_previous_desc, right_previous_desc ;

const double THRESHOLD = 400 ;
bool firstTime = true ;

int k = 2 ;

bool info_matching ;



void imageCallback(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::ImageConstPtr& r_image_msg){

    if(info_matching == false){

        double begin =ros::Time::now().toSec();
        ROS_INFO("Time inici: %f", begin);

    }


    // Variable declaration.
    Ptr<cv::DescriptorMatcher> left_desc_matcher, right_desc_matcher, previous_desc_matcher, current_desc_matcher ;
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr ;
    // cv_bridge::CvImagePtr cv_image_ptr ;

    vector<KeyPoint> left_current_kpts, right_current_kpts ;
    vector<vector<cv::DMatch>> left_vmatches, right_vmatches, previous_vmatches, current_vmatches ;
    vector<cv::DMatch> left_good_matches, right_good_matches, previous_good_matches, current_good_matches ;
    vector<cv::Point2f> left_obj, right_obj, previous_obj, current_obj, left_obj_RANSAC, right_obj_RANSAC, previous_obj_RANSAC, current_obj_RANSAC;
    vector<cv::Point2f> left_scene, right_scene, previous_scene, current_scene, left_scene_RANSAC, right_scene_RANSAC, previous_scene_RANSAC, current_scene_RANSAC;

    int left_good_inliers, right_good_inliers, previous_good_inliers, current_good_inliers ;

    Mat left_RANSACinliersMask, right_RANSACinliersMask, previous_RANSACinliersMask, current_RANSACinliersMask ;
    Mat left_current_desc, right_current_desc ;
    Mat left_out, right_out ;
    Mat left_img, right_img ;
    Mat left_H, right_H, previous_H, current_H ;

    const float ratio_thresh = 0.7f ;

    // Transform image from ROS tu OpenCV.
    try{

        l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8) ;
        r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8) ;

    } catch (cv_bridge::Exception& e) {

        ROS_ERROR("cv_bridge exception: %s", e.what()) ;
        return ;

    }

    left_img = l_cv_ptr -> image ;
    right_img = r_cv_ptr -> image ;

    // Detect keypoints with ORB/SIFT and compute descriptors.

    Ptr<SIFT> sift ;
    //cv::Ptrcv::xfeatures2d::SIFT sift;
    sift = SIFT::create(0,4,0.04,10,1.6);
    sift->detectAndCompute(left_img, Mat(), left_current_kpts, left_current_desc) ;
    sift = SIFT::create(0,4,0.04,10,1.6);
    sift->detectAndCompute(right_img, Mat(), right_current_kpts, right_current_desc) ;
    
    if(firstTime == false){

        // Matching descriptors.
        left_desc_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED) ;
        previous_desc_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED) ;
        right_desc_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED) ;
        current_desc_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED) ;

        if(info_matching == true){

            ROS_INFO("Current left keypoints: %d", left_current_kpts.size()) ;
            ROS_INFO("Previous left keypoints: %d", left_previous_kpts.size()) ;
            ROS_INFO("Current right keypoints: %d", right_current_kpts.size()) ;
            ROS_INFO("Previous right keypoints: %d", right_previous_kpts.size()) ;

            ROS_INFO("-----------------------------------------------") ;

        }


        if ((left_current_kpts.size() >= k) && (left_previous_kpts.size() >= k) && (right_current_kpts.size() >= k) && (right_previous_kpts.size() >= k)){

            left_desc_matcher->knnMatch(left_current_desc, left_previous_desc, left_vmatches, k, cv::noArray(), true) ; 
            previous_desc_matcher->knnMatch(left_previous_desc, right_previous_desc, previous_vmatches, k, cv::noArray(), true) ;
            right_desc_matcher->knnMatch(right_previous_desc, right_current_desc, right_vmatches, k, cv::noArray(), true) ; 
            current_desc_matcher->knnMatch(right_current_desc, left_current_desc, current_vmatches, k, cv::noArray(), true) ;

            /////////////////////////////////////////////////////////////////
            ///////////////////////Lowe's ratio test/////////////////////////
            /////////////////////////////////////////////////////////////////
        
            // Filter matches of the left side using the Lowe's ratio test.
            for (size_t i = 0; i < left_vmatches.size(); i++){
                if (left_vmatches[i][0].distance < ratio_thresh * left_vmatches[i][1].distance){
                    left_good_matches.push_back(left_vmatches[i][0]) ;
                }
            }

            // Filter matches of the previous images using the Lowe's ratio test.
            for (size_t i = 0; i < previous_vmatches.size(); i++){
                if (previous_vmatches[i][0].distance < ratio_thresh * previous_vmatches[i][1].distance){
                    previous_good_matches.push_back(previous_vmatches[i][0]) ;
                }
            }

            // Filter matches of the right side using the Lowe's ratio test.
            for (size_t i = 0; i < right_vmatches.size(); i++){
                if (right_vmatches[i][0].distance < ratio_thresh * right_vmatches[i][1].distance){
                    right_good_matches.push_back(right_vmatches[i][0]) ;
                }
            }

            // Filter matches of the current images using the Lowe's ratio test.
            for (size_t i = 0; i < current_vmatches.size(); i++){
                if (current_vmatches[i][0].distance < ratio_thresh * current_vmatches[i][1].distance){
                    current_good_matches.push_back(current_vmatches[i][0]) ;
                }
            }

            //////////////////////////////////////////////////////////////////
            /////Getting keypoints from good matches of Lowe's ratio test/////
            //////////////////////////////////////////////////////////////////
            
            // Get the keypoints from the good matches of the left side.
            for(int i = 0; i < left_good_matches.size(); i++){
                left_obj.push_back(left_current_kpts[left_good_matches[i].queryIdx].pt) ;
                left_scene.push_back(left_previous_kpts[left_good_matches[i].trainIdx].pt) ;
            }

            // Get the keypoints from the good matches of the previous images.
            for(int i = 0; i < previous_good_matches.size(); i++){
                previous_obj.push_back(left_previous_kpts[previous_good_matches[i].queryIdx].pt) ;
                previous_scene.push_back(right_previous_kpts[previous_good_matches[i].trainIdx].pt) ;
            }

            // Get the keypoints from the good matches of the right side.
            for(int i = 0; i < right_good_matches.size(); i++){
                right_obj.push_back(right_previous_kpts[right_good_matches[i].queryIdx].pt) ;
                right_scene.push_back(right_current_kpts[right_good_matches[i].trainIdx].pt) ; 
            }

            // Get the keypoints from the good matches of the right side.
            for(int i = 0; i < current_good_matches.size(); i++){
                current_obj.push_back(right_current_kpts[current_good_matches[i].queryIdx].pt) ;
                current_scene.push_back(left_current_kpts[current_good_matches[i].trainIdx].pt) ;
            }
        
            //////////////////////////////////////////////////////////////////
            ///////////////Compute homography & apply RANSAC//////////////////
            //////////////////////////////////////////////////////////////////


            // Obtaining homograpghy to apply RANSAC
            try{

                left_H = cv::findHomography(left_obj, left_scene, CV_RANSAC, 3.0, left_RANSACinliersMask) ;
                previous_H = cv::findFundamentalMat(previous_obj, previous_scene, previous_RANSACinliersMask, CV_RANSAC, 3.f) ;
                right_H = cv::findHomography(right_obj, right_scene, CV_RANSAC, 3.0, right_RANSACinliersMask) ;
                current_H = cv::findFundamentalMat(current_obj, current_scene, current_RANSACinliersMask, CV_RANSAC, 3.f) ;

            } catch (cv::Exception& e) {

                ROS_ERROR("cv exception: %s", e.what()) ;

            }

            //////////////////////////////////////////////////////////////////
            /////////Getting keypoints from good matches of RANSAC////////////
            //////////////////////////////////////////////////////////////////

            for(int i = 0; i < left_RANSACinliersMask.rows; i++){
                if(left_RANSACinliersMask.at<bool>(i, 0) == 1){
                    left_obj_RANSAC.push_back(left_obj[i]) ;
                    left_scene_RANSAC.push_back(left_scene[i]) ;
                }
            }

            for(int i = 0; i < previous_RANSACinliersMask.rows; i++){
                if(previous_RANSACinliersMask.at<bool>(i, 0) == 1){
                    previous_obj_RANSAC.push_back(previous_obj[i]) ;
                    previous_scene_RANSAC.push_back(previous_scene[i]) ;
                }
            }

            for(int i = 0; i < right_RANSACinliersMask.rows; i++){
                if(right_RANSACinliersMask.at<bool>(i, 0) == 1){
                    right_obj_RANSAC.push_back(right_obj[i]) ;
                    right_scene_RANSAC.push_back(right_scene[i]) ;
                }
            }

            for(int i = 0; i < current_RANSACinliersMask.rows; i++){
                if(current_RANSACinliersMask.at<bool>(i, 0) == 1){
                    current_obj_RANSAC.push_back(current_obj[i]) ;
                    current_scene_RANSAC.push_back(current_scene[i]) ;
                }
            }

            //////////////////////////////////////////////////////////////////
            /////////Getting keypoints from good matches of RANSAC////////////
            //////////////////////////////////////////////////////////////////

            int cont = 0 ;
            

            for(int i = 0; i < left_obj_RANSAC.size(); i++ ){

                for(int j = 0; j < current_scene_RANSAC.size(); j++){

                    if((left_obj_RANSAC[i].x == current_scene_RANSAC[j].x) && (left_obj_RANSAC[i].y == current_scene_RANSAC[j].y)){
                        cont++ ;
                    }
                }

            }    
           
            //////////////////////////////////////////////////////////////////
            ///////////////////////Debugging zone/////////////////////////////
            //////////////////////////////////////////////////////////////////

            if(info_matching == true){

                left_good_inliers = (int)cv::sum(left_RANSACinliersMask)[0] ;
                previous_good_inliers = (int)cv::sum(previous_RANSACinliersMask)[0] ;
                right_good_inliers = (int)cv::sum(right_RANSACinliersMask)[0] ;
                current_good_inliers = (int)cv::sum(current_RANSACinliersMask)[0] ;

                ROS_INFO("Current left of keypoints: %d", left_current_kpts.size()) ;
                ROS_INFO("Previous left of keypoints: %d", left_previous_kpts.size()) ;
                ROS_INFO("Number of matches: %d", left_vmatches.size()) ;
                ROS_INFO("Number of good matches: %d", left_good_matches.size()) ;
                ROS_INFO("Number of rows after RANSAC: %d", left_RANSACinliersMask.rows) ;
                ROS_INFO("Good matches after RANSAC: %d", left_good_inliers) ;
                ROS_INFO("Left obj RANSAC: %d", left_obj_RANSAC.size()) ;
                ROS_INFO("Left scene RANSAC: %d", left_scene_RANSAC.size()) ;
                
                ROS_INFO("-----------------------------------------------") ;

                // ROS_INFO("Previous left keypoints: %d", left_previous_kpts.size()) ;
                // ROS_INFO("Previous right keypoints: %d", right_previous_kpts.size()) ;
                // ROS_INFO("Number of matches: %d", previous_vmatches.size()) ;
                // ROS_INFO("Number of good matches: %d", previous_good_matches.size()) ;
                // ROS_INFO("Number of rows after RANSAC: %d", previous_RANSACinliersMask.rows) ;
                // ROS_INFO("Good matches after RANSAC: %d", previous_good_inliers) ;
                // ROS_INFO("Left obj RANSAC: %d", left_obj_RANSAC.size()) ;
                // ROS_INFO("Left scene RANSAC: %d", left_scene_RANSAC.size()) ;

                // ROS_INFO("-----------------------------------------------") ;

                // ROS_INFO("Previous right of keypoints: %d", right_previous_kpts.size()) ;
                // ROS_INFO("Current right of keypoints: %d", right_current_kpts.size()) ;
                // ROS_INFO("Number of matches: %d", right_vmatches.size()) ;
                // ROS_INFO("Number of good matches: %d", right_good_matches.size()) ;
                // ROS_INFO("Good matches after RANSAC: %d", right_good_inliers) ;

                // ROS_INFO("-----------------------------------------------") ;

                ROS_INFO("Current right keypoints: %d", right_current_kpts.size()) ;
                ROS_INFO("Current left keypoints: %d", left_current_kpts.size()) ;
                ROS_INFO("Number of matches: %d", current_vmatches.size()) ;
                ROS_INFO("Number of good matches: %d", current_good_matches.size()) ;
                ROS_INFO("Number of rows after RANSAC: %d", current_RANSACinliersMask.rows) ;
                ROS_INFO("Good matches after RANSAC: %d", current_good_inliers) ;
                ROS_INFO("Current obj RANSAC: %d", current_obj_RANSAC.size()) ;
                ROS_INFO("Current scene RANSAC: %d", current_scene_RANSAC.size()) ;
                    
                ROS_INFO("***********************************************") ;
                ROS_INFO("Feature counter at the end of the matching circle: %d", cont) ;
                ROS_INFO("***********************************************") ;

            }
            
        }
    
    }
    

    // Actual keypoints and descriptors turns into previous keypoints and descriptors. 
    left_previous_kpts = left_current_kpts ;
    left_previous_desc = left_current_desc ;
    right_previous_kpts = right_current_kpts ;
    right_previous_desc = right_current_desc ;

    firstTime = false ;

    if(info_matching == false){

        double final =ros::Time::now().toSec(); 
        ROS_INFO("Time final: %f", final);
        ROS_INFO("-----------------------------------------------") ;

    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testSIFT") ;
    ros::NodeHandle nh ;
    ros::NodeHandle nh_private("~") ;

    /* Obtaining constant values by parameter. */
	nh_private.param<bool>("INFO_MATCHING", info_matching, true) ;

    ROS_INFO("INFO_MATCHING = %d", info_matching) ;

    message_filters::Subscriber<sensor_msgs::Image> leftImage_sub(nh, "/stereo_down/left/image_rect", 200) ;
    message_filters::Subscriber<sensor_msgs::Image> rightImage_sub(nh, "/stereo_down/right/image_rect", 200) ;

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol1 ;
    message_filters::Synchronizer<sync_pol1> sync1(sync_pol1(70), leftImage_sub, rightImage_sub) ;

    sync1.registerCallback(boost::bind(&imageCallback, _1, _2)) ;

    ROS_INFO("This is when the SIFT begins") ;

    ros::spin() ;
    return 0 ;
}
