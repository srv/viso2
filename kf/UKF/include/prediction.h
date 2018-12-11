#ifndef PREDICTION_H
#define PREDICTION_H

#include <Eigen/Dense>

#define N_STATES 6

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace Ukf {

class Predictor {
private:
    MatrixXd computeSigmaPoints(VectorXd X, MatrixXd P);
    MatrixXd mapSigmaPoints(VectorXd X, MatrixXd sigma);
    VectorXd predictState(MatrixXd sigma);
    MatrixXd predictCovariance(VectorXd X, MatrixXd sigma);

    MatrixXd P, sigma;
    VectorXd X;
public:
    Predictor();

    void process(VectorXd* X, MatrixXd* P);
};

};


#endif // PREDICTION_H
