viso2
==========
ROS Stack containing a wrapper for libviso2, a visual odometry library. 
http://www.ros.org/wiki/viso2 for the list of contained packages.
***

## Installation

1. Install ROS Melodic: http://wiki.ros.org/ROS/Installation
2. Install Opencv-3.2.0 and Opencv-3.2.0_contrib: https://www.programmersought.com/article/83872474263/ 
3. If there is a problem with libopencv_core.so.3.2 follow the instructions below: https://github.com/cggos/DIPDemoQt/issues/1
4. Download in a catkin workspace the github repository
5. If it is necessary modify the CMakeList.txt to link correctly the libraries from Opencv-3.2.0_contrib
6. Compile the repository

## User guide
# Simple mode

1. Modify stereo_odometer.launch (change the path to the bagfiles or delete them if you are not going to use them, change the name of the cameras to the name of your cameras and change the remap of the topicals).
2. Modify viso2_parameters.yaml
3. Change the disparity parameters to yours
4. Run the stereo odometer using: roslaunch viso2_ros stereo_odometer.launch

# Automatic mode

1. Modify stereo_odometer_auto.launch (change the path to the bagfiles or delete them if you are not going to use them, change the name of the cameras to the name of your cameras and change the remap of the topicals).
2. Modify viso2_parameters_auto.yaml
3. Change the disparity parameters to yours
4. Run the script experiment_automator.py: python3 experiment_automator.py --out_path=folder/to/save/the/results
