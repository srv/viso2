#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>
#include <iostream>

#include "../../tools/include/tools.h"

#define pi 3.14159265359

const double kappa = 1;   /* observations noise covariance constant */
const double alpha = 0.1; /* state noise covariance constant */

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
private:
  /* Initial state and covariance values */
  VectorXd X_;
  MatrixXd P_;

  /* (A,B,C,D) system matrices */
  MatrixXd A;
  MatrixXd B;
  MatrixXd H;

  /* State vector, covariance matrix and Kalman Filter Gain */
  VectorXd X;
  MatrixXd P;
  MatrixXd K;

  /* Model and observations noise */
  MatrixXd R;
  MatrixXd Q;

  /* State dimension */
  long int n;

  /* Predict and correct steps (invoked by process()) */
  void predict(const VectorXd& U);
  void correct(const VectorXd& Z, double dt);

public:
  /* Kalman filter initializer (X,P) */
  KalmanFilter(const VectorXd& X0, const MatrixXd& P0);
  /* Function that receives (A,B,H) in each iteration */
  void system(const MatrixXd& A, const MatrixXd& B,
              const MatrixXd& H);
  /* Funtion that invokes all the methods */
  bool process(const VectorXd& Z, const VectorXd& U, double dt);
  /* Function to reset covariance matrix in case of rotation */
  void resetP();
  /* Function that returns the state */
  VectorXd getState() const;
};

#endif // KALMAN_FILTER_H
