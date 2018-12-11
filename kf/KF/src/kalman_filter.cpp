#include "../include/kalman_filter.h"

KalmanFilter::KalmanFilter(const VectorXd& X0, const MatrixXd& P0)
{
  (*this).X_ = X0;
  (*this).P_ = P0;

  n = X0.rows();
  R = MatrixXd::Identity(n, n) * alpha;
}

void KalmanFilter::system(const MatrixXd& A, const MatrixXd& B,
                          const MatrixXd& H)
{
  (*this).A = A;
  (*this).B = B;
  (*this).H = H;
}

void KalmanFilter::predict(const VectorXd& U)
{
  X    = A * X_ + B * U;
  X[0] = normalizeAngle(X[0]);

  P = A * P_ * A.transpose() + R;
}

void KalmanFilter::correct(const VectorXd& Z, double dt)
{
  const MatrixXd I = MatrixXd::Identity(n, n);

  long int l = Z.rows();
  /* construction of observations noise covariance matrix */
  double q_w = (abs(X_[4] * dt * 180 / pi) < 1)
                   ? alpha
                   : alpha / (abs(X_[4] * dt * 180 / pi));
  Q = MatrixXd(l, l);
  Q << 10, 0, 0, 0, q_w, 0, 0, 0, q_w;

  K    = (P * H.transpose()) * (H * P * H.transpose() + Q).inverse();
  X    = X + K * (Z - H * X);
  X[0] = normalizeAngle(X[0]);
  P    = (I - K * H) * P;

  X_ = X;
  P_ = P;

  std::cout << "K:\n" << K << std::endl << std::endl;
  std::cout << "P:\n" << P << std::endl << std::endl;
  std::cout << "Q:\n" << Q << std::endl << std::endl;
  std::cout << "R:\n" << R << std::endl << std::endl;
}

bool KalmanFilter::process(const VectorXd& Z, const VectorXd& U,
                           double dt)
{
  if (U.rows() != B.cols() || Z.rows() != H.rows() ||
      A.rows() != X_.rows() || A.cols() != X_.rows() ||
      B.rows() != X_.rows() || H.cols() != X_.rows() ||
      P_.rows() != X_.rows() || P_.cols() != X_.rows())
    return 1;

  predict(U);
  correct(Z, dt);

  return 0;
}

VectorXd KalmanFilter::getState() const { return (*this).X; }

void KalmanFilter::resetP()
{
  P(1, 3) = 0;
  P(2, 4) = 0;
  P(3, 1) = 0;
  P(4, 2) = 0;
}
