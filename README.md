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
roslaunch viso2_ros perspective_version.launch
roslaunch viso2_ros omni_version.launch
```
**NOTE:** We provide the **camera calibration file** for the fisheye camera ('data/calib_results.txt'). You have to set the correct path to the rosbag file in both launch files and the calibration text file in the fisheye.launch. 
We also provide a **rviz configuration file** to easy visualization of the results.

## If you use it, please cite:
```
@Article{app9245516,
AUTHOR = {Aguiar, André and Santos, Filipe and Sousa, Armando Jorge and Santos, Luís},
TITLE = {FAST-FUSION: An Improved Accuracy Omnidirectional Visual Odometry System with Sensor Fusion and GPU Optimization for Embedded Low Cost Hardware},
JOURNAL = {Applied Sciences},
VOLUME = {9},
YEAR = {2019},
NUMBER = {24},
ARTICLE-NUMBER = {5516},
URL = {https://www.mdpi.com/2076-3417/9/24/5516},
ISSN = {2076-3417},
ABSTRACT = {The main task while developing a mobile robot is to achieve accurate and robust navigation in a given environment. To achieve such a goal, the ability of the robot to localize itself is crucial. In outdoor, namely agricultural environments, this task becomes a real challenge because odometry is not always usable and global navigation satellite systems (GNSS) signals are blocked or significantly degraded. To answer this challenge, this work presents a solution for outdoor localization based on an omnidirectional visual odometry technique fused with a gyroscope and a low cost planar light detection and ranging (LIDAR), that is optimized to run in a low cost graphical processing unit (GPU). This solution, named FAST-FUSION, proposes to the scientific community three core contributions. The first contribution is an extension to the state-of-the-art monocular visual odometry (Libviso2) to work with omnidirectional cameras and single axis gyro to increase the system accuracy. The second contribution, it is an algorithm that considers low cost LIDAR data to estimate the motion scale and solve the limitations of monocular visual odometer systems. Finally, we propose an heterogeneous computing optimization that considers a Raspberry Pi GPU to improve the visual odometry runtime performance in low cost platforms. To test and evaluate FAST-FUSION, we created three open-source datasets in an outdoor environment. Results shows that FAST-FUSION is acceptable to run in real-time in low cost hardware and that outperforms the original Libviso2 approach in terms of time performance and motion estimation accuracy.},
DOI = {10.3390/app9245516}
}

```
and/or
* 
```
@InProceedings{10.1007/978-3-030-35990-4_11,
author="Aguiar, Andr{\'e}
and Santos, Filipe
and Santos, Lu{\'i}s
and Sousa, Armando",
editor="Silva, Manuel F.
and Lu{\'i}s Lima, Jos{\'e}
and Reis, Lu{\'i}s Paulo
and Sanfeliu, Alberto
and Tardioli, Danilo",
title="A Version of Libviso2 for Central Dioptric Omnidirectional Cameras with a Laser-Based Scale Calculation",
booktitle="Robot 2019: Fourth Iberian Robotics Conference",
year="2020",
publisher="Springer International Publishing",
address="Cham",
pages="127--138",
abstract="Monocular Visual Odometry techniques represent a challenging and appealing research area in robotics navigation field. The use of a single camera to track robot motion is a hardware-cheap solution. In this context, there are few Visual Odometry methods on the literature that estimate robot pose accurately using a single camera without any other source of information. The use of omnidirectional cameras in this field is still not consensual. Many works show that for outdoor environments the use of them does represent an improvement compared with the use of conventional perspective cameras. Besides that, in this work we propose an open-source monocular omnidirectional version of the state-of-the-art method Libviso2 that outperforms the original one even in outdoor scenes. This approach is suitable for central dioptric omnidirectional cameras and takes advantage of their wider field of view to calculate the robot motion with a really positive performance on the context of monocular Visual Odometry. We also propose a novel approach to calculate the scale factor that uses matches between laser measures and 3-D triangulated feature points to do so. The novelty of this work consists in the association of the laser ranges with the features on the omnidirectional image. Results were generate using three open-source datasets built in-house showing that our unified system largely outperforms the original monocular version of Libviso2.",
isbn="978-3-030-35990-4"
}
```

## Acknowledges

This work is co-financed by the ERDF – European Regional Development Fund through the Operational Programme for Competitiveness and Internationalisation - COMPETE 2020 under the PORTUGAL 2020 Partnership Agreement, and through the Portuguese National Innovation Agency (ANI) as a part of project «ROMOVI: POCI-01-0247-FEDER-017945»".
