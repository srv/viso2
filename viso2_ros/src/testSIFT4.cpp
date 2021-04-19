/*
 *  Created on: April 2021
 *  Author: Bo Miquel Nordfeldt Fiol
 *  Name: testSIFT4
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


#include <cv_bridge/cv_bridge.h>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d.hpp>

// #include <opencv2/calib3d.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/objdetect.hpp>
// #include <opencv2/features2d.hpp>
// #include <opencv2/calib3d.hpp> 
// #include <opencv2/xfeatures2d/nonfree.hpp>

using namespace sensor_msgs;
using namespace message_filters; 

using std::vector ;

using cv::Mat ;
using cv::KeyPoint ;
using cv::Ptr ;

using cv::xfeatures2d::SIFT ;


/////////////////////////////////////////////////////////////////
////////////////Global variable declaration//////////////////////
/////////////////////////////////////////////////////////////////

vector<KeyPoint> l_pre_kpts, r_pre_kpts ;
Mat l_pre_desc, r_pre_desc ;

bool firstTime = true ;

double Time_ImageCurrent ;
double Time_ImagePrevious ;

int k = 2 ;

struct auxiliar_return {

    vector<KeyPoint> kpts1 ;
    vector<KeyPoint> kpts2 ;
    Mat desc1 ;
    Mat desc2 ;
    vector<cv::Point2f> coord1 ;
    vector<cv::Point2f> coord2 ;
    bool correct ;

};

typedef struct auxiliar_return Struct;


/////////////////////////////////////////////////////////////////
/////////////////////Auxiliar functions//////////////////////////
/////////////////////////////////////////////////////////////////

Struct SIFTmatching(vector<KeyPoint> kpts1, vector<KeyPoint> kpts2, Mat desc1, Mat desc2, bool homography) {

    // Structure
    Struct s ;

    // Pointers
    Ptr<cv::DescriptorMatcher> matcher ;

    // Vectors
    vector<vector<cv::DMatch>> matches ;
    vector<KeyPoint> kpts1_aft_match, kpts2_aft_match ;
    vector<KeyPoint> kpts1_aft_H, kpts2_aft_H ;

    // Matrix
    Mat desc1_aft_match, desc2_aft_match ;
    Mat H, F ;
    Mat RANSACinliersMask ;
    Mat desc1_aft_H, desc2_aft_H ;

    // Point2f
    vector<cv::Point2f> coord1_aft_match, coord2_aft_match ;
    vector<cv::Point2f> coord1_aft_H, coord2_aft_H ;

    // Iterators
    int i ;

    // Compute the matchings
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED) ;
    matcher->knnMatch(desc1, desc2, matches, k, cv::noArray(), true) ;

    // Obtaining keypoints, coordinates and descriptors after matching 
    for(i = 0; i < matches.size(); i++){

        kpts1_aft_match.push_back(kpts1[matches[i][0].queryIdx]) ;
        desc1_aft_match.push_back(desc1.row(matches[i][0].queryIdx)) ;
        coord1_aft_match.push_back(kpts1[matches[i][0].queryIdx].pt) ;
        
        kpts2_aft_match.push_back(kpts2[matches[i][0].trainIdx]) ;
        desc2_aft_match.push_back(desc2.row(matches[i][0].trainIdx)) ;
        coord2_aft_match.push_back(kpts2[matches[i][0].trainIdx].pt) ;

    }

    // Compute Homography or fundamental matrix
    if(homography == true){

        try{

            H = cv::findHomography(coord1_aft_match, coord2_aft_match, CV_RANSAC, 3.0, RANSACinliersMask) ;

        } catch (cv::Exception& e) {

            ROS_ERROR("cv exception: %s", e.what()) ;
            s.correct = false ;
            return s ;

        }

    } else {

        try{

            F = cv::findFundamentalMat(coord1_aft_match, coord2_aft_match, RANSACinliersMask, CV_RANSAC, 3.f) ;

        } catch (cv::Exception& e) {

            ROS_ERROR("cv exception: %s", e.what()) ;
            s.correct = false ;
            return s ;

        }

    }

    // Obtaining keypoints, coordinates and descriptors after homography
    for(i = 0; i < RANSACinliersMask.rows; i++){
        if(RANSACinliersMask.at<bool>(i, 0) == 1){

            kpts1_aft_H.push_back(kpts1_aft_match[i]) ;
            desc1_aft_H.push_back(desc1_aft_match.row(i)) ;
            coord1_aft_H.push_back(coord1_aft_match[i]) ;

            kpts2_aft_H.push_back(kpts2_aft_match[i]) ;
            desc2_aft_H.push_back(desc2_aft_match.row(i)) ;
            coord2_aft_H.push_back(coord2_aft_match[i]) ;

        }
    }

    s.kpts1 = kpts1_aft_H ;
    s.kpts2 = kpts2_aft_H ;
    s.desc1 = desc1_aft_H ;
    s.desc2 = desc2_aft_H ;
    s.coord1 = coord1_aft_H ;
    s.coord2 = coord2_aft_H ;
    s.correct = true ;

    return s ;

}

/////////////////////////////////////////////////////////////////
/////////////////////////Callback////////////////////////////////
/////////////////////////////////////////////////////////////////

void imageCallback(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::ImageConstPtr& r_image_msg){

    Time_ImageCurrent = ros::Time::now().toSec() ;

    /////////////////////////////////////////////////////////////////
    /////////////////Local variable declaration//////////////////////
    /////////////////////////////////////////////////////////////////

    //Vectors
    vector<KeyPoint> l_curr_kpts, r_curr_kpts ;

    // Matrix
    Mat left_img, right_img ;

    Mat l_curr_desc, r_curr_desc ;

    // Pointers
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr ;
    Ptr<SIFT> sift ;

    // Structures
    Struct left_matching, previous_matching, right_matching, current_matching ;

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

        if ((l_curr_kpts.size() >= k) && (l_pre_kpts.size() >= k) && (r_curr_kpts.size() >= k) && (r_pre_kpts.size() >= k)){

            ROS_INFO("Left current keypoints before prueba: %d", l_curr_kpts.size()) ;
            ROS_INFO("Left previous keypoints before prueba: %d", l_pre_kpts.size()) ;
            ROS_INFO("Left current descriptos before prueba: %d", l_curr_desc.size()) ;
            ROS_INFO("Left previous descriptos before prueba: %d", l_pre_desc.size()) ;

            left_matching = SIFTmatching(l_curr_kpts, l_pre_kpts, l_curr_desc, l_pre_desc, true) ;

            if(left_matching.correct == false){

                l_pre_kpts = l_curr_kpts ;
                l_pre_desc = l_curr_desc ;
                r_pre_kpts = r_curr_kpts ;
                r_pre_desc = r_curr_desc ;

                Time_ImagePrevious = Time_ImageCurrent ;
                return ;

            }

            ROS_INFO("Left current keypoints after H: %d", left_matching.kpts1.size()) ;
            ROS_INFO("Left previous keypoints after H: %d", left_matching.kpts2.size()) ;
            ROS_INFO("Left current descriptors after H: %d", left_matching.desc1.size()) ;
            ROS_INFO("Left previous descriptors after H: %d", left_matching.desc2.size()) ;
            ROS_INFO("Left current coordinates after H: %d", left_matching.coord1.size()) ;
            ROS_INFO("Left previous coordinates after H: %d", left_matching.coord2.size()) ;
            ROS_INFO("-----------------------------------------------") ;

            previous_matching = SIFTmatching(left_matching.kpts2, r_pre_kpts, left_matching.desc2, r_pre_desc, false) ;

            if(previous_matching.correct == false){

                l_pre_kpts = l_curr_kpts ;
                l_pre_desc = l_curr_desc ;
                r_pre_kpts = r_curr_kpts ;
                r_pre_desc = r_curr_desc ;

                Time_ImagePrevious = Time_ImageCurrent ;
                return ;

            }

            ROS_INFO("Left previous keypoints after F: %d", previous_matching.kpts1.size()) ;
            ROS_INFO("Right previous keypoints after F: %d", previous_matching.kpts2.size()) ;
            ROS_INFO("Left previous descriptors after F: %d", previous_matching.desc1.size()) ;
            ROS_INFO("Right previous descriptors after F: %d", previous_matching.desc2.size()) ;
            ROS_INFO("Left previous coordinates after F: %d", previous_matching.coord1.size()) ;
            ROS_INFO("Right previous coordinates after F: %d", previous_matching.coord2.size()) ;
            ROS_INFO("-----------------------------------------------") ;

            right_matching = SIFTmatching(previous_matching.kpts2, r_curr_kpts, previous_matching.desc2, r_curr_desc, true) ;

            if(right_matching.correct == false){

                l_pre_kpts = l_curr_kpts ;
                l_pre_desc = l_curr_desc ;
                r_pre_kpts = r_curr_kpts ;
                r_pre_desc = r_curr_desc ;

                Time_ImagePrevious = Time_ImageCurrent ;
                return ;

            }

            ROS_INFO("Right previous keypoints after H: %d", right_matching.kpts1.size()) ;
            ROS_INFO("Right current keypoints after H: %d", right_matching.kpts2.size()) ;
            ROS_INFO("Right previous descriptors after H: %d", right_matching.desc1.size()) ;
            ROS_INFO("Right current descriptors after H: %d", right_matching.desc2.size()) ;
            ROS_INFO("Right previous coordinates after H: %d", right_matching.coord1.size()) ;
            ROS_INFO("Right current coordinates after H: %d", right_matching.coord2.size()) ;
            ROS_INFO("-----------------------------------------------") ;

            current_matching = SIFTmatching(right_matching.kpts2, left_matching.kpts1, right_matching.desc2, left_matching.desc1, false) ;

            if(current_matching.correct == false){

                l_pre_kpts = l_curr_kpts ;
                l_pre_desc = l_curr_desc ;
                r_pre_kpts = r_curr_kpts ;
                r_pre_desc = r_curr_desc ;

                Time_ImagePrevious = Time_ImageCurrent ;
                return ;

            }

            ROS_INFO("Right current keypoints after F: %d", current_matching.kpts1.size()) ;
            ROS_INFO("Left current keypoints after F: %d", current_matching.kpts2.size()) ;
            ROS_INFO("Right current descriptors after F: %d", current_matching.desc1.size()) ;
            ROS_INFO("Left current descriptors after F: %d", current_matching.desc2.size()) ;
            ROS_INFO("Right current coordinates after F: %d", current_matching.coord1.size()) ;
            ROS_INFO("Left current coordinates after F: %d", current_matching.coord2.size()) ;
            ROS_INFO("-----------------------------------------------") ;
            // ROS_INFO("***********************************************") ;

            /////////////////////////////////////////////////////////////////
            ///////////////Obtaining previous coordinates////////////////////
            /////////////////////////////////////////////////////////////////

            int cont = 0 ;
            std::vector<int> cont_j1 ; 
            std::vector<int> cont_j2 ;
            bool save_cont1 = false ;
            bool save_cont2 = false ;

            int i, j ;

            for(i = 0; i < current_matching.coord2.size(); i++){
                for(j = 0; j < left_matching.coord1.size(); j++){
                    if((current_matching.coord2[i].x == left_matching.coord1[j].x) && (current_matching.coord2[i].y == left_matching.coord1[j].y)){
                        if (std::find(std::begin(cont_j1), std::end(cont_j1), j) == std::end(cont_j1)){
                            save_cont1 = true ;
                            cont_j1.push_back(j) ;
                            // save l_pre_coord_aft_H[j].x -- left_matching.coord2[j].x
                            // save l_pre_coord_aft_H[j].y -- left_matching.coord2[j].y
                            break ;
                        }                    
                    }
                }
                for(j = 0; j < right_matching.coord2.size(); j++){
                    if((current_matching.coord1[i].x == right_matching.coord2[j].x) && (current_matching.coord1[i].y == right_matching.coord2[j].y)){
                        if (std::find(std::begin(cont_j2), std::end(cont_j2), j) == std::end(cont_j2)){
                            save_cont2 = true ;
                            cont_j2.push_back(j) ;
                            // save r_pre_coord_after_H[j].x -- right_matching.coord1[j].x
                            // save r_pre_coord_after_H[j].y -- right_matching.coord1[j].y
                            break ;
                        }   
                    }
                }

                if(save_cont1 == true && save_cont2 == true){

                    cont++ ;
                    save_cont1 = false ;
                    save_cont2 = false ;

                    //p_matched_2.push_back(l_curr_coord_aft_F[i].x)
                    //p_matched_2.push_back(l_curr_coord_aft_F[i].y)
                    //p_matched_2.push_back(r_curr_coord_aft_F[i].x)
                    //p_matched_2.push_back(r_curr_coord_aft_F[i].y)
                    //p_matched_2.push_back(l_pre_coord_aft_H[j].x)
                    //p_matched_2.push_back(l_pre_coord_aft_H[j].y)
                    //p_matched_2.push_back(r_pre_coord_aft_H[j].x)
                    //p_matched_2.push_back(r_pre_coord_aft_H[i].y)

                }else{

                    save_cont1 = false ;
                    save_cont2 = false ;

                }

            }

            // ROS_INFO("l_curr_coord_aft_H.size(): %d", l_curr_coord_aft_H.size()) ;
            ROS_INFO("Contador: %d", cont) ;
            ROS_INFO("Time between process: %f", Time_ImageCurrent - Time_ImagePrevious) ;
            ROS_INFO("***********************************************") ;


        } else {

            ROS_INFO("There isn't enough keypoints");
            ROS_INFO("***********************************************") ;

        }

    }

    firstTime = false ;

    // Actual keypoints and descriptors turns into previous keypoints and descriptors. 
    l_pre_kpts = l_curr_kpts ;
    l_pre_desc = l_curr_desc ;
    r_pre_kpts = r_curr_kpts ;
    r_pre_desc = r_curr_desc ;

    Time_ImagePrevious = Time_ImageCurrent ;

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

    ROS_INFO("This is when the SIFT4 begins") ;

    ros::spin() ;
    return 0 ;
}