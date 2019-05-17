
#ifndef VISO_MONO_omnidirectional_H
#define VISO_MONO_omnidirectional_H

#include "viso.h"

class VisualOdometryMonoOmnidirectional : public VisualOdometry
{
public:

  // monocular-specific parameters (mandatory: height,pitch)
  struct parameters : public VisualOdometry::parameters {
    double                      height;           // camera height above ground (meters)
    double                      pitch;            // camera pitch (rad, negative=pointing down)
    int32_t                     ransac_iters;     // number of RANSAC iterations
    double                      inlier_threshold; // fundamental matrix inlier threshold
    double                      motion_threshold; // directly return false on small motions
    parameters () {
      height           = 1.0;
      pitch            = 0.0;
      ransac_iters     = 2000;
      inlier_threshold = 0.00001;
      motion_threshold = 100.0;
    }
  };

  // constructor, takes as input a parameter structure
  VisualOdometryMonoOmnidirectional (parameters param);

  // deconstructor
  ~VisualOdometryMonoOmnidirectional ();

  // process a new image, pushs the image back to an internal ring buffer.
  // valid motion estimates are available after calling process for two times.
  // inputs: I ......... pointer to rectified image (uint8, row-aligned)
  //         dims[0] ... width of I
  //         dims[1] ... height of I
  //         dims[2] ... bytes per line (often equal to width)
  //         replace ... replace current image with I, without copying last current
  //                     image to previous image internally. this option can be used
  //                     when small/no motions are observed to obtain Tr_delta wrt
  //                     an older coordinate system / time step than the previous one.
  // output: returns false if motion too small or an error occured
  bool process (uint8_t *I, int32_t* dims, bool replace = false);


private:
  template<class T> struct idx_cmp {
    idx_cmp(const T arr) : arr(arr) {}
    bool operator()(const size_t a, const size_t b) const { return arr[a] < arr[b]; }
    const T arr;
  };  

  std::vector<double>  estimateMotion (std::vector<Matcher::p_match> p_matched);  
  Matrix               smallerThanMedian (Matrix &X,double &median);
  void                 fundamentalMatrix (const std::vector<Matcher::p_match_3d> &p_matched_3d,const std::vector<int32_t> &active,Matrix &F);
  void                 EtoRt(Matrix &E,std::vector<Matcher::p_match_3d> &p_matched_3d,Matrix &X,Matrix &R,Matrix &t);
  int32_t              triangulateChieral (std::vector<Matcher::p_match_3d> &p_matched_3d,Matrix &R,Matrix &t,Matrix &X);
  std::vector<int32_t> getInlier (std::vector<Matcher::p_match_3d> &p_matched_3d,Matrix &F);

  // functions that project a pixel point into a 3D unit sphere point
  // and a 3D world point into a pixel point  
  void                 world2cam(double point2D[2], double point3D[3]);
  void                 cam2world(double point3D[3], double point2D[2]);
  Matcher::p_match_3d  projectIntoUnitSphere(Matcher::p_match);
  Matcher::p_match     projectIntoImage(Matrix X, Matrix P1, Matrix P2);
  std::vector<Matcher::p_match_3d> projectMatches(std::vector<Matcher::p_match> matches);
  std::vector<Matcher::p_match> reprojectMatches(Matrix X, Matrix P1, Matrix P2);

  // parameters
  parameters param;  
};

#endif

