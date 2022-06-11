/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#ifndef VISO_STEREO_H
#define VISO_STEREO_H

#include "viso.h"

#include </usr/local/include/opencv2/core/core.hpp>
using cv::Mat ;


class VisualOdometryStereo : public VisualOdometry {

public:

  // stereo-specific parameters (mandatory: base)
  struct parameters : public VisualOdometry::parameters {
    double  base;             // baseline (meters)
    int32_t ransac_iters;     // number of RANSAC iterations
    double  inlier_threshold; // fundamental matrix inlier threshold
    bool    reweighting;      // lower border weights (more robust to calibration errors)
    parameters () {
      base             = 1.0;
      ransac_iters     = 200;
      inlier_threshold = 2.0;
      reweighting      = true;
    }
  };

  // constructor, takes as inpute a parameter structure
  VisualOdometryStereo (parameters param);
  
  // deconstructor
  ~VisualOdometryStereo ();
  
  // process a new images, push the images back to an internal ring buffer.
  // valid motion estimates are available after calling process for two times.
  // inputs: I1 ........ pointer to rectified left image (uint8, row-aligned)
  //         I2 ........ pointer to rectified right image (uint8, row-aligned)
  //         dims[0] ... width of I1 and I2 (both must be of same size)
  //         dims[1] ... height of I1 and I2 (both must be of same size)
  //         dims[2] ... bytes per line (often equal to width)
  //         replace ... replace current images with I1 and I2, without copying last current
  //                     images to previous images internally. this option can be used
  //                     when small/no motions are observed to obtain Tr_delta wrt
  //                     an older coordinate system / time step than the previous one.
  // output: returns false if an error occured
  bool process (uint8_t *I1,uint8_t *I2,int32_t* dims,bool replace=false);
  
  //BMNF: 
  /**********************************************************************************************************
  Process a new images, push the images back, compute the new match circle, do a bucketing if is required and
  updates motion.

  Parameters:
  @left_img:  Opencv matrix that contains the left image.
  @right_img: Opencv matrix that contains the right image.
  @replace: Boolean that allows to calculated the circle match if its value is "false". If its value is 
            "true" the information of current images is transformed to information of previous images.
  @bucketing: Boolean that allows the bucketing
  @combination: Integer that defines which combination of feature detector and descriptor is being used.
  @nOctaveLayers: Number of layers per octave.
  @contrastThreshold_SIFT: The contrast threshold used to filter out weak features in semi-uniform
                           (low-contrast) regions. The larger the threshold, the less features are produced by the detector.
  @edgeThreshold_SIFT: The threshold used to filter out edge-like features. Note that the its meaning is
                       different from the contrastThreshold, i.e. the larger the edgeThreshold, the less features are filtered
                       out (more features are retained).
  @sigma_SIFT: The sigma of the Gaussian applied to the input image at the octave #0. 
  @hessianThreshold_SURF: Threshold for hessian keypoint detector used in SURF.
  @nOctaves_SURF: Number of pyramid octaves the keypoint detector will use. 
  @homography_reprojThreshold: Constrain to calculate the homography.
  @epipolar_constrain: Constraint to calculate the fundamental matrix.
  @contrast_threshold: Parameter for SIFT feature detector

  Returns:
  @updateMotion: Boolean that returns if the motion update is correct or if an error occured.
  ***********************************************************************************************************/
  bool new_process(Mat left_img, Mat right_img, bool replace, bool bucketing, int combination, int nOctaveLayers,
                  double contrastThreshold_SIFT, double edgeThreshold_SIFT, double sigma_SIFT, 
                  double hessianThreshold_SURF, int nOctaves_SURF, 
                  double homography_reprojThreshold, int epipolar_constrain) ; 

  using VisualOdometry::process;



private:

  std::vector<double>  estimateMotion (std::vector<Matcher::p_match> p_matched,double cameraHeight, bool mono_odometry);
  enum                 result { UPDATED, FAILED, CONVERGED };  
  result               updateParameters(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps);
  void                 computeObservations(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active);
  void                 computeResidualsAndJacobian(std::vector<double> &tr,std::vector<int32_t> &active);
  std::vector<int32_t> getInlier(std::vector<Matcher::p_match> &p_matched,std::vector<double> &tr);

  double *X,*Y,*Z;    // 3d points
  double *p_residual; // residuals (p_residual=p_observe-p_predict)
  
  // parameters
  parameters param;
};

#endif // VISO_STEREO_H

