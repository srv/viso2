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
#include <iostream>
#include <string.h>
#include "matcher.h"

using namespace std;

static Matcher *M;

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
    if (nrhs!=1+1)
      mexErrMsgTxt("1 input required (param).");
    
    // check if we have a parameter structure
    if (!mxIsStruct(prhs[1]))
      mexErrMsgTxt("Input param must be a structure.");    
    
    // parameters
    Matcher::parameters param;

    // for all fields of parameter structure overwrite parameters
    for (int32_t i=0; i<mxGetNumberOfFields(prhs[1]); i++) {
      const char *field_name = mxGetFieldNameByNumber(prhs[1],i);
      mxArray    *field_val  = mxGetFieldByNumber(prhs[1],0,i);

      // value
      if (mxIsDouble(field_val)) {
        double val = *((double*)mxGetPr(field_val));
        if (!strcmp(field_name,"nms_n"))                  param.nms_n = val;
        if (!strcmp(field_name,"nms_tau"))                param.nms_tau = val;
        if (!strcmp(field_name,"match_binsize"))          param.match_binsize = val;
        if (!strcmp(field_name,"match_radius"))           param.match_radius = val;
        if (!strcmp(field_name,"match_disp_tolerance"))   param.match_disp_tolerance = val;
        if (!strcmp(field_name,"outlier_disp_tolerance")) param.outlier_disp_tolerance = val;
        if (!strcmp(field_name,"outlier_flow_tolerance")) param.outlier_flow_tolerance = val;
        if (!strcmp(field_name,"multi_stage"))            param.multi_stage = val;
        if (!strcmp(field_name,"half_resolution"))        param.half_resolution = val;
        if (!strcmp(field_name,"refinement"))             param.refinement = val;
      }
    }
    
    // create matcher instance
    M = new Matcher(param);

  // close
  } else if (!strcmp(command,"close")) {
    delete M;
    
  // push back stereo image pair
  } else if (!strcmp(command,"push") || !strcmp(command,"replace")) {
    
    // check arguments
    if (nrhs!=1+1 && nrhs!=1+2)
      mexErrMsgTxt("1 or 2 inputs required (I1=left image,I2=right image).");
    if (!mxIsUint8(prhs[1]) || mxGetNumberOfDimensions(prhs[1])!=2)
      mexErrMsgTxt("Input I1 (left image) must be a uint8_t matrix.");
    
    // get pointer to left image
    uint8_t* I1          = (uint8_t*)mxGetPr(prhs[1]);
    const int32_t *dims1 = mxGetDimensions(prhs[1]);
    
    // transpose
    uint8_t* I1_         = transpose<uint8_t>(I1,dims1);
    int32_t  dims1_[]    = {dims1[1],dims1[0],dims1[1]};
    
    // push back single image
    if (nrhs==1+1) {
      
      // compute features and put them to ring buffer
      M->pushBack(I1_,dims1_,!strcmp(command,"replace"));
    
    // push back stereo image pair
    } else {
      
      if (!mxIsUint8(prhs[2]) || mxGetNumberOfDimensions(prhs[2])!=2)
        mexErrMsgTxt("Input I2 (right image) must be a uint8_t matrix.");
      
      // get pointer to right image
      uint8_t* I2          = (uint8_t*)mxGetPr(prhs[2]);
      const int32_t *dims2 = mxGetDimensions(prhs[2]);
      
      // transpose
      uint8_t* I2_         = transpose<uint8_t>(I2,dims2);
      int32_t  dims2_[]    = {dims2[1],dims2[0],dims2[1]};

      // check image size
      if (dims1_[0]!=dims2_[0] || dims1_[1]!=dims2_[1])
        mexErrMsgTxt("Input I1 and I2 must be images of same size.");

      // compute features and put them to ring buffer
      M->pushBack(I1_,I2_,dims1_,!strcmp(command,"replace"));
      
      // free temporary memory
      free(I2_);
    }
    
    // free temporary memory
    free(I1_);

  // match features
  } else if (!strcmp(command,"match")) {
    
    // check arguments
    if (nrhs!=1+1 && nrhs!=1)
      mexErrMsgTxt("1 input required (method).");
    if (nlhs!=0)
      mexErrMsgTxt("No output required.");
    
    if (!mxIsDouble(prhs[1]) || mxGetM(prhs[1])*mxGetN(prhs[1])!=1)
        mexErrMsgTxt("Input method must be a double scalar.");
    
    // matching method: 0 = flow, 1 = stereo, 2 = quad matching
    int32_t method  =  (int32_t)*((double*)mxGetPr(prhs[1]));
    
    // do matching
    M->matchFeatures(method);
    
  // bucketing
  } else if (!strcmp(command,"bucketing")) {
    
    // check arguments
    if (nrhs!=1+3)
      mexErrMsgTxt("3 inputs required (max_features,bucket_width,bucket_height).");
    if (nlhs!=0)
      mexErrMsgTxt("No output required.");
    
    // remove closeby features via bucketing
    if (!mxIsDouble(prhs[1]) || mxGetM(prhs[1])*mxGetN(prhs[1])!=1)
      mexErrMsgTxt("Input max_features must be a double scalar.");
    if (!mxIsDouble(prhs[2]) || mxGetM(prhs[2])*mxGetN(prhs[2])!=1)
      mexErrMsgTxt("Input bucket_width must be a double scalar.");
    if (!mxIsDouble(prhs[3]) || mxGetM(prhs[3])*mxGetN(prhs[3])!=1)
      mexErrMsgTxt("Input bucket_height must be a double scalar.");

    // get pointers to input data
    int32_t max_features  =  (int32_t)*((double*)mxGetPr(prhs[1]));
    float   bucket_width  =           *((double*)mxGetPr(prhs[2]));
    float   bucket_height =           *((double*)mxGetPr(prhs[3]));

    // bucketing
    M->bucketFeatures(max_features,bucket_width,bucket_height);
    
  // get matches
  } else if (!strcmp(command,"get_matches")) {
    
    // check arguments
    if (nrhs!=1+1)
      mexErrMsgTxt("1 input required (method).");
    if (nlhs!=1)
      mexErrMsgTxt("One output required (p_matched).");
    
    if (!mxIsDouble(prhs[1]) || mxGetM(prhs[1])*mxGetN(prhs[1])!=1)
        mexErrMsgTxt("Input method must be a double scalar.");
    
    // matching method: 0 = flow, 1 = stereo, 2 = quad matching
    int32_t method  =  (int32_t)*((double*)mxGetPr(prhs[1]));
    
    // grab matches
    vector<Matcher::p_match> matches = M->getMatches();
    
    // method: flow
    if (method==0) {
      
      // create output matrix with matches
      const int32_t p_matched_dims[] = {4,matches.size()};
      plhs[0] = mxCreateNumericArray(2,p_matched_dims,mxDOUBLE_CLASS,mxREAL);
      double* p_matched_mex = (double*)mxGetPr(plhs[0]);

      // copy matches to mex array (convert indices: C++ => MATLAB)
      int32_t k=0;
      for (vector<Matcher::p_match>::iterator it=matches.begin(); it!=matches.end(); it++) {
        *(p_matched_mex+k++) = it->u1p+1.0;
        *(p_matched_mex+k++) = it->v1p+1.0;
        *(p_matched_mex+k++) = it->u1c+1.0;
        *(p_matched_mex+k++) = it->v1c+1.0;
      }

    // method: stereo
    } else if (method==1) {
      
      // create output matrix with matches
      const int32_t p_matched_dims[] = {4,matches.size()};
      plhs[0] = mxCreateNumericArray(2,p_matched_dims,mxDOUBLE_CLASS,mxREAL);
      double* p_matched_mex = (double*)mxGetPr(plhs[0]);

      // copy matches to mex array (convert indices: C++ => MATLAB)
      int32_t k=0;
      for (vector<Matcher::p_match>::iterator it=matches.begin(); it!=matches.end(); it++) {
        *(p_matched_mex+k++) = it->u1c+1.0;
        *(p_matched_mex+k++) = it->v1c+1.0;
        *(p_matched_mex+k++) = it->u2c+1.0;
        *(p_matched_mex+k++) = it->v2c+1.0;
      }

    // method: quad matching
    } else {
      
      // create output matrix with matches
      const int32_t p_matched_dims[] = {8,matches.size()};
      plhs[0] = mxCreateNumericArray(2,p_matched_dims,mxDOUBLE_CLASS,mxREAL);
      double* p_matched_mex = (double*)mxGetPr(plhs[0]);

      // copy matches to mex array (convert indices: C++ => MATLAB)
      int32_t k=0;
      for (vector<Matcher::p_match>::iterator it=matches.begin(); it!=matches.end(); it++) {
        *(p_matched_mex+k++) = it->u1p+1.0;
        *(p_matched_mex+k++) = it->v1p+1.0;
        *(p_matched_mex+k++) = it->u2p+1.0;
        *(p_matched_mex+k++) = it->v2p+1.0;
        *(p_matched_mex+k++) = it->u1c+1.0;
        *(p_matched_mex+k++) = it->v1c+1.0;
        *(p_matched_mex+k++) = it->u2c+1.0;
        *(p_matched_mex+k++) = it->v2c+1.0;
      }
    }
    
  // get matching indices
  } else if (!strcmp(command,"get_indices")) {
    
    // check arguments
    if (nrhs!=1+1)
      mexErrMsgTxt("1 input required (method).");
    if (nlhs!=1)
      mexErrMsgTxt("One output required (i_matched).");
    if (!mxIsDouble(prhs[1]) || mxGetM(prhs[1])*mxGetN(prhs[1])!=1)
        mexErrMsgTxt("Input method must be a double scalar.");
    
    // matching method: 0 = flow, 1 = stereo, 2 = quad matching
    int32_t method  =  (int32_t)*((double*)mxGetPr(prhs[1]));
    
    // grab matches
    vector<Matcher::p_match> matches = M->getMatches();
    
    // method: flow
    if (method==0) {
      
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

    // method: stereo
    } else if (method==1) {
      
      // create output matrix with matches
      const int32_t i_matched_dims[] = {2,matches.size()};
      plhs[0] = mxCreateNumericArray(2,i_matched_dims,mxDOUBLE_CLASS,mxREAL);
      double* i_matched_mex = (double*)mxGetPr(plhs[0]);

      // copy matches to mex array
      int32_t k=0;
      for (vector<Matcher::p_match>::iterator it=matches.begin(); it!=matches.end(); it++) {
        *(i_matched_mex+k++) = it->i1c+1;
        *(i_matched_mex+k++) = it->i2c+1;
      }

    // method: quad matching
    } else {
      
      // create output matrix with matches
      const int32_t i_matched_dims[] = {4,matches.size()};
      plhs[0] = mxCreateNumericArray(2,i_matched_dims,mxDOUBLE_CLASS,mxREAL);
      double* i_matched_mex = (double*)mxGetPr(plhs[0]);

      // copy matches to mex array
      int32_t k=0;
      for (vector<Matcher::p_match>::iterator it=matches.begin(); it!=matches.end(); it++) {
        *(i_matched_mex+k++) = it->i1p+1;
        *(i_matched_mex+k++) = it->i2p+1;
        *(i_matched_mex+k++) = it->i1c+1;
        *(i_matched_mex+k++) = it->i2c+1;
      }
    }

  // unknown command
  } else {
    mexPrintf("Unknown command: %s\n",command);
  }
}
