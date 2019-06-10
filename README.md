viso2
==========
ROS Stack containing a wrapper for libviso2, a visual odometry library. 
http://www.ros.org/wiki/viso2 for the list of contained packages.
***

## Omnidirectional version

* This package contains an additional version of the original Libviso2 that works with a **monocular omnidirectional camera system**.

### Dependencies

* The only additional dependency of this version is the camera calibration toolbox [Ocamcalib](https://sites.google.com/site/scarabotix/ocamcalib-toolbox) for Matlab.
  - Instead of using a	 normal pinhole camera calibration model, the method uses the unified camera model proposed by Davide Scaramuzza.
  - Download the toolbox from [here](https://sites.google.com/site/scarabotix/ocamcalib-toolbox/ocamcalib-toolbox-download-page), put it in your Matlab workspace and execute the `ocam_calib` command to run the application.
  - After calibrating the camera save the generated text file that will be used by the method.

### How to execute

* To execute omnidirectional Libviso2 make sure you have a camera node publishing over ROS.
* After that you can use the demo launch file that we provide in the following way:
```
roslaunch viso2_ros demo.launch
```
* Make sure that first you set the **file calibration path** correctly by changing the `calib_path` parameter.
* Also you can edit/add the default parameters that Libviso2 use (don't forget that camera height and pitch are mandatory to scale calculation).

### Performance test

* If you want to check the performance of the method before using it in your own system you can use the bag file that we provide [here](https://drive.google.com/file/d/1FfBOMPrdk-uqLeNm3NNGXJQxdkKqgUfI/view?usp=sharing).

##### Available topics on the bag file
1. `perspective_cam/image/compressed` contains a perspective camera sequence that can be used by the default monocular Libviso2 version to benchmark.
2. `fisheye_cam/image/compressed` contains the same sequence captured by a fisheye camera.
3. `slam_out_pose` contains the camera motion estimation computed by Hector SLAM method.
4. `/husky_velocity_controller/odom` constains conventional odometry motion estimation.
5. `/fix` constains GPS data.
6. `/map` contains a map of the environment designed using a laser.
7. (...) and others.

##### Running the test sequences

* To run both test sequences you can execute the following commands after setting the proper paths for the bag file and the calibration file.
```
roslaunch viso2_ros perspective.launch
roslaunch viso2_ros fisheye.launch
```
**NOTE:** We provide the **camera calibration file** for the fisheye camera ('data/calib_results.txt'). You have to set the correct path to the rosbag file in both launch files and the calibration text file in the fisheye.launch. 
We also provide a **rviz configuration file** to easy visualization of the results.

## Acknowledges

This work is co-financed by the ERDF – European Regional Development Fund through the Operational Programme for Competitiveness and Internationalisation - COMPETE 2020 under the PORTUGAL 2020 Partnership Agreement, and through the Portuguese National Innovation Agency (ANI) as a part of project «ROMOVI: POCI-01-0247-FEDER-017945»".
