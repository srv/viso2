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

* If you want to check the performance of the method before using it in your own system you can use the bag file that we provide.

##### Available topics on the bag file
1. `perspective_cam/image` contains a perspective camera sequence that can be used by the default monocular Libviso2 version to benchmark.
2. `fisheye_cam/image` contains the same sequence captured by a fisheye camera.
3. `ground_truth` contains the real camera motion computed by Hector SLAM method.

##### Running the test sequences

* To run both test sequences you can execute the following commands.
```
roslaunch viso2_ros perspective.launch
roslaunch viso2_ros fisheye.launch
```
**NOTE:** We provide the camera calibration file for the fisheye camera.

