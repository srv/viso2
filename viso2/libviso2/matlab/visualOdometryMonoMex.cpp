/*
Copyright 2012. All rights reserved.
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

#include "mex.h"
#include "viso_mono.h"

#include <iostream>
#include <string.h>

using namespace std;

static VisualOdometryMono *viso;

template<class T> T* transpose(T* I,const int32_t* dims) {
  T* I_ = (T*)malloc(dims[0]*dims[1]*sizeof(T));
  for (int32_t u=0; u<dims[1]; u++) {
    for (int32_t v=0; v<dims[0]; v++) {
      I_[v*dims[1]+u] = *I;
      I++;
    }
  }
  return I_;
}

void mexFunction (int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {

  // read command
  char command[128];
  mxGetString(prhs[0],command,128);

  // init
  if (!strcmp(command,"init")) {
    
    // check arguments
    if(nrhs!=1+1)
      mexErrMsgTxt("1 parameter required (param).");

    // check if we have a parameter structure
    if (!mxIsStruct(prhs[1]))
      mexErrMsgTxt("Input param must be a structure.");
    
    // parameters
    VisualOdometryMono::parameters param;

    // for all fields of parameter structure overwrite parameters
    for (int32_t i=0; i<mxGetNumberOfFields(prhs[1]); i++) {
      const char *field_name = mxGetFieldNameByNumber(prhs[1],i);
      mxArray    *field_val  = mxGetFieldByNumber(prhs[1],0,i);

      // value
      if (mxIsDouble(field_val)) {
        double val = *((double*)mxGetPr(field_val));
        if (!strcmp(field_name,"nms_n"))                  param.match.nms_n = val;
        if (!strcmp(field_name,"nms_tau"))                param.match.nms_tau = val;
        if (!strcmp(field_name,"match_binsize"))          param.match.match_binsize = val;
        if (!strcmp(field_name,"match_radius"))           param.match.match_radius = val;
        if (!strcmp(field_name,"match_disp_tolerance"))   param.match.match_disp_tolerance = val;
        if (!strcmp(field_name,"outlier_disp_tolerance")) param.match.outlier_disp_tolerance = val;
        if (!strcmp(field_name,"outlier_flow_tolerance")) param.match.outlier_flow_tolerance = val;
        if (!strcmp(field_name,"multi_stage"))            param.match.multi_stage = val;
        if (!strcmp(field_name,"half_resolution"))        param.match.half_resolution = val;
        if (!strcmp(field_name,"refinement"))             param.match.refinement = val;
        if (!strcmp(field_name,"max_features"))           param.bucket.max_features = val;
        if (!strcmp(field_name,"bucket_width"))           param.bucket.bucket_width = val;
        if (!strcmp(field_name,"bucket_height"))          param.bucket.bucket_height = val;
        if (!strcmp(field_name,"f"))                      param.calib.f = val;
        if (!strcmp(field_name,"cu"))                     param.calib.cu = val;
        if (!strcmp(field_name,"cv"))                     param.calib.cv = val;
        if (!strcmp(field_name,"height"))                 param.height = val;
        if (!strcmp(field_name,"pitch"))                  param.pitch = val;
        if (!strcmp(field_name,"ransac_iters"))           param.ransac_iters = val;
        if (!strcmp(field_name,"inlier_threshold"))       param.inlier_threshold = val;
        if (!strcmp(field_name,"motion_threshold"))       param.motion_threshold = val;
      }
    }
    
    // create visual odometry instance
    viso = new VisualOdometryMono(param);

  // process
  } else if (!strcmp(command,"process")) {
    
    // check for proper number of arguments
    if (nrhs!=1+1 && nrhs!=2+1)
      mexErrMsgTxt("1 or 2 inputs required: I (image), [replace].");
    if (nlhs!=1) 
      mexErrMsgTxt("1 outputs required: Tr (transformation from previous to current frame.");  

    // check for proper argument types and sizes
    if (!mxIsUint8(prhs[1]) || mxGetNumberOfDimensions(prhs[1])!=2)
      mexErrMsgTxt("Input I (image) must be a uint8 image.");
    if (nrhs==2+1 && !mxIsDouble(prhs[2]))
      mexErrMsgTxt("Input replace must be a double scalar (0/1).");
    
    // get image pointers
    uint8_t*       I    = (uint8_t*)mxGetPr(prhs[1]);
    const int32_t *dims =   mxGetDimensions(prhs[1]);
    
    // set replacement variable
    bool replace = false;
    if (nrhs==2+1)
      replace = (bool)(*((double*)mxGetPr(prhs[2])));
    
    // transpose images (align row-wise)
    uint8_t* I_      = transpose<uint8_t>(I,dims);
    int32_t  dims_[] = {dims[1],dims[0],dims[1]};
        
    // compute visual odometry, return motion estimate on success
    // (mapping from previous to current frame)
    if (viso->process(I_,dims_,replace)) {

      Matrix Tr_delta = ~(viso->getMotion());
      const int tr_dims[] = {4,4};
      plhs[0] = mxCreateNumericArray(2,tr_dims,mxDOUBLE_CLASS,mxREAL);
      Tr_delta.getData((double*)mxGetPr(plhs[0]));
      
    // return empty transformation on failure
    // (motion too small to estimate scale)
    } else {
      
      // return motion estimate (mapping from previous to current frame)
      const int tr_dims[] = {0,0};
      plhs[0] = mxCreateNumericArray(2,tr_dims,mxDOUBLE_CLASS,mxREAL);     
    }
    
    // release temporary memory
    free(I_);
    
  // query number of matches
  } else if (!strcmp(command,"num_matches")) {
    const int dims[] = {1,1};
    plhs[0] = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL);
    *((double*)mxGetPr(plhs[0])) = (double)viso->getNumberOfMatches();
    
  // query number of inliers
  } else if (!strcmp(command,"num_inliers")) {
    const int dims[] = {1,1};
    plhs[0] = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL);
    *((double*)mxGetPr(plhs[0])) = (double)viso->getNumberOfInliers();
    
  // get matches
  } else if (!strcmp(command,"get_matches")) {
    
    // check arguments
    if (nrhs!=1+0)
      mexErrMsgTxt("No input required.");
    if (nlhs!=1)
      mexErrMsgTxt("One output required (p_matched).");
   
    // grab matches
    vector<Matcher::p_match> matches = viso->getMatches();
          
    // create output matrix with matches
    const int32_t p_matched_dims[] = {4,matches.size()};
    plhs[0] = mxCreateNumericArray(2,p_matched_dims,mxDOUBLE_CLASS,mxREAL);
    double* p_matched_mex = (double*)mxGetPr(plhs[0]);

    // copy matches to mex array (convert indices: C++ => MATLAB)
    int32_t k=0;
    for (vector<Matcher::p_match>::iterator it=matches.begin(); it!=matches.end(); it++) {
      *(p_matched_mex+k++) = it->u1p+1;
      *(p_matched_mex+k++) = it->v1p+1;
      *(p_matched_mex+k++) = it->u1c+1;
      *(p_matched_mex+k++) = it->v1c+1;
    }
    
  // get matching indices
  } else if (!strcmp(command,"get_indices")) {
    
    // check arguments
    if (nrhs!=1+0)
      mexErrMsgTxt("No input required.");
    if (nlhs!=1)
      mexErrMsgTxt("One output required (i_matched).");
    
    // grab matches
    vector<Matcher::p_match> matches = viso->getMatches();

    // create output matrix with matches
    const int32_t i_matched_dims[] = {2,matches.size()};
    plhs[0] = mxCreateNumericArray(2,i_matched_dims,mxDOUBLE_CLASS,mxREAL);
    double* i_matched_mex = (double*)mxGetPr(plhs[0]);

    // copy matches to mex array
    int32_t k=0;
    for (vector<Matcher::p_match>::iterator it=matches.begin(); it!=matches.end(); it++) {
      *(i_matched_mex+k++) = it->i1p+1;
      *(i_matched_mex+k++) = it->i1c+1;
    }
    
  // close
  } else if (!strcmp(command,"close")) {
    delete viso;
  
  // unknown command
  } else {
    mexPrintf("Unknown command: %s\n",command);
  }
}
