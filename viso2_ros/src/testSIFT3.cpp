/*
 *  Created on: 2021
 *  Author: Bo Miquel Nordfeldt Fiol
 *  Name: testSIFT2
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

using cv::xfeatures2d::SIFT ;

using namespace sensor_msgs;
using namespace message_filters; 


/////////////////////////////////////////////////////////////////
////////////////Global variable declaration//////////////////////
/////////////////////////////////////////////////////////////////

vector<KeyPoint> l_pre_kpts, r_pre_kpts ;
Mat l_pre_desc, r_pre_desc ;

bool firstTime = true ;

int k = 2 ;

/////////////////////////////////////////////////////////////////
/////////////////////////Callback////////////////////////////////
/////////////////////////////////////////////////////////////////

void imageCallback(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::ImageConstPtr& r_image_msg){


    /////////////////////////////////////////////////////////////////
    /////////////////Local variable declaration//////////////////////
    /////////////////////////////////////////////////////////////////

    // Vectors
    vector<vector<cv::DMatch>> left_matches, right_matches, previous_matches, current_vmatches ;

    vector<KeyPoint> l_curr_kpts, r_curr_kpts ;

    vector<KeyPoint> l_curr_kpts_aft_match, l_pre_kpts_aft_match ;
    vector<KeyPoint> l_curr_kpts_aft_H, l_pre_kpts_aft_H ;

    vector<KeyPoint> l_pre_kpts_aft_H_aft_previous_match, r_pre_kpts_aft_match ;
    vector<KeyPoint> l_pre_kpts_aft_F, r_pre_kpts_aft_F ;

    vector<KeyPoint> r_pre_kpts_aft_F_aft_right_match, r_curr_kpts_aft_match ;
    vector<KeyPoint> r_pre_kpts_aft_H, r_curr_kpts_aft_H ;

    // Matrix
    Mat left_img, right_img ;

    Mat l_curr_desc, r_curr_desc ;

    Mat l_curr_desc_aft_match, l_pre_desc_aft_match ;
    Mat left_H ;
    Mat l_curr_desc_aft_H, l_pre_desc_aft_H ;    
    Mat left_RANSACinliersMask ;

    Mat l_pre_desc_aft_H_aft_previous_match, r_pre_desc_aft_match ;
    Mat previous_F ;
    Mat l_pre_desc_aft_F, r_pre_desc_aft_F ;
    Mat previous_RANSACinliersMask ;

    Mat r_pre_desc_aft_F_aft_right_match, r_curr_desc_aft_match ;
    Mat right_H ;
    Mat r_pre_desc_aft_H, r_curr_desc_aft_H ;
    Mat right_RANSACinliersMask ;

    // Pointers
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr ;
    Ptr<SIFT> sift ;
    Ptr<cv::DescriptorMatcher> left_desc_matcher, right_desc_matcher, previous_desc_matcher, current_desc_matcher ;

    // Point2f
    vector<cv::Point2f> l_curr_coord_aft_match, l_pre_coord_aft_match ;
    vector<cv::Point2f> l_curr_coord_aft_H, l_pre_coord_aft_H ;

    vector<cv::Point2f> l_pre_coord_aft_H_aft_previous_match, r_pre_coord_aft_match ;
    vector<cv::Point2f> l_pre_coord_aft_F, r_pre_coord_aft_F ;

    vector<cv::Point2f> r_pre_coord_aft_F_aft_right_match, r_curr_coord_aft_match ;
    vector<cv::Point2f> r_pre_coord_aft_H, r_curr_coord_aft_H ;

    /////////////////////////////////////////////////////////////////
    ////////////Transform image from ROS tu OpenCV///////////////////
    /////////////////////////////////////////////////////////////////

    try{

        l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8) ;
        r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8) ;

    } catch (cv_bridge::Exception& e) {

        ROS_ERROR("cv_bridge exception: %s", e.what()) ;
        return ;

    }

    left_img = l_cv_ptr -> image ;
    right_img = r_cv_ptr -> image ;


    /////////////////////////////////////////////////////////////////
    ////////////////////////Compute SIFT/////////////////////////////
    /////////////////////////////////////////////////////////////////

    sift = SIFT::create(0, 3, 0.04, 10, 1.6) ;
    sift->detectAndCompute(left_img, Mat(), l_curr_kpts, l_curr_desc) ;
    sift = SIFT::create(0, 3, 0.04, 10, 1.6) ;
    sift->detectAndCompute(right_img, Mat(), r_curr_kpts, r_curr_desc) ;

    

    if(firstTime == false){


        /////////////////////////////////////////////////////////////////
        //////////////////////////Left side//////////////////////////////
        /////////////////////////////////////////////////////////////////

        // Compute the matchings of the left side
        left_desc_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED) ;
        left_desc_matcher->knnMatch(l_curr_desc, l_pre_desc, left_matches, k, cv::noArray(), true) ;

        // Obtaining keypoints, coordinates and descriptors of the left side after matching 
        for(int i = 0; i < left_matches.size(); i++){

            l_curr_kpts_aft_match.push_back(l_curr_kpts[left_matches[i][0].queryIdx]) ;
            l_curr_desc_aft_match.push_back(l_curr_desc.row(left_matches[i][0].queryIdx)) ;
            l_curr_coord_aft_match.push_back(l_curr_kpts[left_matches[i][0].queryIdx].pt) ;
            
            l_pre_kpts_aft_match.push_back(l_pre_kpts[left_matches[i][0].trainIdx]) ;
            l_pre_desc_aft_match.push_back(l_pre_desc.row(left_matches[i][0].trainIdx)) ;
            l_pre_coord_aft_match.push_back(l_pre_kpts[left_matches[i][0].trainIdx].pt) ;

        }

        ROS_INFO("Current left keypoints: %d", l_curr_kpts.size()) ;
        ROS_INFO("Previous left keypoints: %d", l_pre_kpts.size()) ;
        ROS_INFO("Current left keypoints after matches: %d", l_curr_kpts_aft_match.size()) ;
        ROS_INFO("Previous left matches keypoints: %d", l_pre_kpts_aft_match.size()) ;
        ROS_INFO("Current left matches descriptors: %d", l_curr_desc_aft_match.size()) ;
        ROS_INFO("Previous left matches descriptors: %d", l_pre_desc_aft_match.size()) ;

        // Compute left Homography
        try{

            left_H = cv::findHomography(l_curr_coord_aft_match, l_pre_coord_aft_match, CV_RANSAC, 3.0, left_RANSACinliersMask) ;

        } catch (cv::Exception& e) {

            ROS_ERROR("cv exception: %s", e.what()) ;

        }

        // Obtaining keypoints, coordinates and descriptors after homography
        for(int i = 0; i < left_RANSACinliersMask.rows; i++){
            if(left_RANSACinliersMask.at<bool>(i, 0) == 1){

                l_curr_kpts_aft_H.push_back(l_curr_kpts_aft_match[i]) ;
                l_curr_desc_aft_H.push_back(l_curr_desc_aft_match.row(i)) ;
                l_curr_coord_aft_H.push_back(l_curr_coord_aft_match[i]) ;

                l_pre_kpts_aft_H.push_back(l_pre_kpts_aft_match[i]) ;
                l_pre_desc_aft_H.push_back(l_pre_desc_aft_match.row(i)) ;
                l_pre_coord_aft_H.push_back(l_pre_coord_aft_match[i]) ;

            }
        }

        ROS_INFO("Current left keypoints after H: %d", l_curr_kpts_aft_H.size()) ;
        ROS_INFO("Previous left keypoints after H: %d", l_pre_kpts_aft_H.size()) ;
        ROS_INFO("-----------------------------------------------") ;


        /////////////////////////////////////////////////////////////////
        ////////////////////////Previous side////////////////////////////
        /////////////////////////////////////////////////////////////////

        // Compute the matchings of the previous side
        previous_desc_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED) ;
        previous_desc_matcher->knnMatch(l_pre_desc_aft_H, r_pre_desc, previous_matches, k, cv::noArray(), true) ;

        // Obtaining keypoints, coordinates and descriptors of the previous side after matching 
        for(int i = 0; i < previous_matches.size(); i++){

            l_pre_kpts_aft_H_aft_previous_match.push_back(l_pre_kpts_aft_H[previous_matches[i][0].queryIdx]) ;
            l_pre_desc_aft_H_aft_previous_match.push_back(l_pre_desc_aft_H.row(previous_matches[i][0].queryIdx)) ;
            l_pre_coord_aft_H_aft_previous_match.push_back(l_pre_kpts_aft_H[previous_matches[i][0].queryIdx].pt) ;

            r_pre_kpts_aft_match.push_back(r_pre_kpts[previous_matches[i][0].trainIdx]) ;
            r_pre_desc_aft_match.push_back(r_pre_desc.row(previous_matches[i][0].trainIdx)) ;
            r_pre_coord_aft_match.push_back(r_pre_kpts[previous_matches[i][0].trainIdx].pt) ;
 
        }

        ROS_INFO("Previous left keypoints after H: %d", l_pre_kpts_aft_H.size()) ;
        ROS_INFO("Previous right keypoints: %d", r_pre_kpts.size()) ;
        ROS_INFO("Previous left keypoints after H and after match : %d", l_pre_kpts_aft_H_aft_previous_match.size() ) ;
        ROS_INFO("Previous right keypoints after match: %d", r_pre_kpts_aft_match.size()) ;

        // Compute previous Fundamental Matrix
        try{

            previous_F = cv::findFundamentalMat(l_pre_coord_aft_H_aft_previous_match, r_pre_coord_aft_match, previous_RANSACinliersMask, CV_RANSAC, 3.f) ;

        } catch (cv::Exception& e) {

            ROS_ERROR("cv exception: %s", e.what()) ;

        }

        // Obtaining keypoints, coordinates and descriptors of the previous side after fundamental matrix 
        for(int i = 0; i < previous_RANSACinliersMask.rows; i++){
            if(previous_RANSACinliersMask.at<bool>(i, 0) == 1){


                l_pre_kpts_aft_F.push_back(l_pre_kpts_aft_H_aft_previous_match[i]) ;
                l_pre_desc_aft_F.push_back(l_pre_desc_aft_H_aft_previous_match.row(i)) ;
                l_pre_coord_aft_F.push_back(l_pre_coord_aft_H_aft_previous_match[i]) ;

                r_pre_kpts_aft_F.push_back(r_pre_kpts_aft_match[i]) ;
                r_pre_desc_aft_F.push_back(r_pre_desc_aft_match.row(i)) ;
                r_pre_coord_aft_F.push_back(r_pre_coord_aft_match[i]) ;

            }
        }

        ROS_INFO("Previous left keypoints after F: %d", l_pre_kpts_aft_F.size()) ;
        ROS_INFO("Previous right keypoints after F: %d", r_pre_kpts_aft_F.size()) ;
        ROS_INFO("-----------------------------------------------") ;

        /////////////////////////////////////////////////////////////////
        /////////////////////////Right side//////////////////////////////
        /////////////////////////////////////////////////////////////////

        // Compute the matchings of the right side
        right_desc_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED) ;
        right_desc_matcher->knnMatch(r_pre_desc_aft_F, r_curr_desc, right_matches, k, cv::noArray(), true) ;

        // Obtaining keypoints, coordinates and descriptors of the right side after matching 
        for(int i = 0; i < right_matches.size(); i++){

            r_pre_kpts_aft_F_aft_right_match.push_back(r_pre_kpts_aft_F[right_matches[i][0].queryIdx]) ;
            r_pre_desc_aft_F_aft_right_match.push_back(r_pre_desc_aft_F.row(right_matches[i][0].queryIdx)) ;
            r_pre_coord_aft_F_aft_right_match.push_back(r_pre_kpts_aft_F[right_matches[i][0].queryIdx].pt) ;

            r_curr_kpts_aft_match.push_back(r_curr_kpts[right_matches[i][0].trainIdx]) ;
            r_curr_desc_aft_match.push_back(r_curr_desc.row(right_matches[i][0].trainIdx)) ;
            r_curr_coord_aft_match.push_back(r_curr_kpts[right_matches[i][0].trainIdx].pt) ;
 
        }
        
        ROS_INFO("Previous right keypoints after F: %d", r_pre_kpts_aft_F.size()) ;
        ROS_INFO("Current right keypoints: %d", r_curr_kpts.size()) ;
        ROS_INFO("Previous left keypoints after H and after match : %d", r_pre_kpts_aft_F_aft_right_match.size() ) ;
        ROS_INFO("Previous right keypoints after match: %d", r_curr_kpts_aft_match.size()) ;

        // Compute right Homography
        try{

            right_H = cv::findHomography(r_pre_coord_aft_F_aft_right_match, r_curr_coord_aft_match, CV_RANSAC, 3.0, right_RANSACinliersMask) ;

        } catch (cv::Exception& e) {

            ROS_ERROR("cv exception: %s", e.what()) ;

        }

        // Obtaining keypoints, coordinates and descriptors of the right side after homography
        for(int i = 0; i < right_RANSACinliersMask.rows; i++){
            if(right_RANSACinliersMask.at<bool>(i, 0) == 1){

                r_pre_kpts_aft_H.push_back(r_pre_kpts_aft_F_aft_right_match[i]) ;
                r_pre_desc_aft_H.push_back(r_pre_desc_aft_F_aft_right_match.row(i)) ;
                r_pre_coord_aft_H.push_back(r_pre_coord_aft_F_aft_right_match[i]) ;

                r_curr_kpts_aft_H.push_back(r_curr_kpts_aft_match[i]) ;
                r_curr_desc_aft_H.push_back(r_curr_desc_aft_match.row(i)) ;
                r_curr_coord_aft_H.push_back(r_curr_coord_aft_match[i]) ;

            }
        }

        ROS_INFO("Previous right keypoints after H: %d", r_pre_kpts_aft_H.size()) ;
        ROS_INFO("Current right keypoints after H: %d", r_curr_kpts_aft_H.size()) ;
        ROS_INFO("***********************************************") ;

        /////////////////////////////////////////////////////////////////
        ///////////////////////Current side//////////////////////////////
        /////////////////////////////////////////////////////////////////

        // Compute the matchings of the current side
        right_desc_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED) ;
        right_desc_matcher->knnMatch(r_pre_desc_aft_F, r_curr_desc, right_matches, k, cv::noArray(), true) ;



    }

    firstTime = false ;


    // Actual keypoints and descriptors turns into previous keypoints and descriptors. 
    l_pre_kpts = l_curr_kpts ;
    l_pre_desc = l_curr_desc ;
    r_pre_kpts = r_curr_kpts ;
    r_pre_desc = r_curr_desc ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testSIFT") ;
    ros::NodeHandle nh ;
    ros::NodeHandle nh_private("~") ;

    message_filters::Subscriber<sensor_msgs::Image> leftImage_sub(nh, "/stereo_down/left/image_rect", 200) ;
    message_filters::Subscriber<sensor_msgs::Image> rightImage_sub(nh, "/stereo_down/right/image_rect", 200) ;

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol1 ;
    message_filters::Synchronizer<sync_pol1> sync1(sync_pol1(70), leftImage_sub, rightImage_sub) ;

    sync1.registerCallback(boost::bind(&imageCallback, _1, _2)) ;

    ROS_INFO("This is when the SIFT3 begins") ;

    ros::spin() ;
    return 0 ;
}