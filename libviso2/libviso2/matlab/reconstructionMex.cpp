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
#include "reconstruction.h"

using namespace std;

static Reconstruction *R;

void mexFunction (int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {

  // read command
  char command[128];
  mxGetString(prhs[0],command,128);

  // init
  if (!strcmp(command,"init")) {
    
    // check arguments
    if(nrhs!=1+3)
      mexErrMsgTxt("3 parameters required (f,cu,cv).");
    if(!mxIsDouble(prhs[1]) || mxGetM(prhs[1])*mxGetN(prhs[1])!=1)
      mexErrMsgTxt("Input f must be a double scalar.");
    if(!mxIsDouble(prhs[2]) || mxGetM(prhs[2])*mxGetN(prhs[2])!=1)
      mexErrMsgTxt("Input cu must be a double scalar.");
    if(!mxIsDouble(prhs[3]) || mxGetM(prhs[3])*mxGetN(prhs[3])!=1)
      mexErrMsgTxt("Input cv must be a double scalar.");
    
    double f    = *((double*)mxGetPr(prhs[1]));
    double cu   = *((double*)mxGetPr(prhs[2]));
    double cv   = *((double*)mxGetPr(prhs[3]));

    R = new Reconstruction();
    R->setCalibration(f,cu,cv);

  // close
  } else if (!strcmp(command,"close")) {
    delete R;

  // update via observations
  } else if (!strcmp(command,"update")) {
    
    // check arguments (for parameter description see reconstruction.h)
    if(nrhs!=1+7)
      mexErrMsgTxt("7 inputs required (p_matched,i_matched,Tr,point_type,min_track_length,max_dist,min_angle).");
    if(!mxIsDouble(prhs[1]) || mxGetM(prhs[1])!=4)
      mexErrMsgTxt("Input p_matched must be a 4xN double matrix.");
    if(!mxIsDouble(prhs[2]) || mxGetM(prhs[2])!=2)
      mexErrMsgTxt("Input i_matched must be a 2xN double matrix.");
    if(!mxIsDouble(prhs[3]) || mxGetM(prhs[3])!=4 || mxGetN(prhs[3])!=4)
      mexErrMsgTxt("Input Tr must be a 4x4 double matrix.");
    if(!mxIsDouble(prhs[4]) || mxGetM(prhs[4])*mxGetN(prhs[4])!=1)
      mexErrMsgTxt("Input point_type must be a double scalar.");
    if(!mxIsDouble(prhs[5]) || mxGetM(prhs[5])*mxGetN(prhs[5])!=1)
      mexErrMsgTxt("Input min_track_length must be a double scalar.");
    if(!mxIsDouble(prhs[6]) || mxGetM(prhs[6])*mxGetN(prhs[6])!=1)
      mexErrMsgTxt("Input max_dist must be a double scalar.");
    if(!mxIsDouble(prhs[7]) || mxGetM(prhs[7])*mxGetN(prhs[7])!=1)
      mexErrMsgTxt("Input min_angle must be a double scalar.");
    if(nlhs!=0)
      mexErrMsgTxt("No outputs required.");
    
    // get pointers
    double* p_matched_data   =            (double*)mxGetPr(prhs[1]);
    double* i_matched_data   =            (double*)mxGetPr(prhs[2]);
    double* Tr_data          =            (double*)mxGetPr(prhs[3]);
    int32_t point_type       = (int32_t)*((double*)mxGetPr(prhs[4]));
    int32_t min_track_length = (int32_t)*((double*)mxGetPr(prhs[5]));
    double  max_dist         =          *((double*)mxGetPr(prhs[6]));
    double  min_angle        =          *((double*)mxGetPr(prhs[7]));
    int     N                =                      mxGetN(prhs[1]);
    
    // create match vector (convert indices: MATLAB => C++)
    vector<Matcher::p_match> p_matched;    
    Matcher::p_match match;
    int32_t k1=0,k2=0;
    for (int i=0; i<N; i++) {
      match.u1p =          p_matched_data[k1++]-1;
      match.v1p =          p_matched_data[k1++]-1;
      match.i1p = (int32_t)i_matched_data[k2++]-1;
      match.u1c =          p_matched_data[k1++]-1;
      match.v1c =          p_matched_data[k1++]-1;
      match.i1c = (int32_t)i_matched_data[k2++]-1;
      p_matched.push_back(match);
    }
    
    // create matrix
    Matrix Tr(4,4,Tr_data);
    Tr = ~Tr;
    
    // update
    R->update(p_matched,Tr,point_type,min_track_length,max_dist,min_angle);
    
  // get points
  } else if (!strcmp(command,"getpoints")) {
    
    // check arguments
    if(nlhs!=1)
      mexErrMsgTxt("1 output required (points).");
    
    // get inliers
    vector<Reconstruction::point3d> points = R->getPoints();
    const int dims[] = {3,points.size()};
    plhs[0] = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL);

    double *points_mex = (double*)mxGetPr(plhs[0]);
    int32_t k=0;
    for (int i=0; i<points.size(); i++) {
      points_mex[k++] = points[i].x;
      points_mex[k++] = points[i].y;
      points_mex[k++] = points[i].z;
    }
    
  // unknown command
  } else {
    mexPrintf("Unknown command: %s\n",command);
  }
  
}
