//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_POSITIONVELOCITYESTIMATOR_H
#define ZQ_HUMANOID_POSITIONVELOCITYESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"
#include <Utilities/Timer.h>

/*!
 * Position and velocity estimator based on a Kalman Filter.
 */
template <typename T>
class LinearKFPositionVelocityEstimator : public GenericEstimator<T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LinearKFPositionVelocityEstimator();
    virtual void run();
    virtual void setup();

private:
    Eigen::Matrix<T, 12, 1> _xhat;
    Eigen::Matrix<T, 6, 1> _ps;
    Eigen::Matrix<T, 6, 1> _vs;
    Eigen::Matrix<T, 12, 12> _A;
    Eigen::Matrix<T, 12, 12> _Q0;
    Eigen::Matrix<T, 12, 12> _P;
    Eigen::Matrix<T, 14, 14> _R0;
    Eigen::Matrix<T, 12, 3> _B;
    Eigen::Matrix<T, 14, 12> _C;
    Vec3<T> dp_f_pre;
    Vec3<T> omega_body_pre;
    Vec3<T> acc_body_pre;
    bool first_run = true;
    Vec3<T> init_pos;
    Timer timer_obs;
};

/*!
 * "Cheater" position and velocity estimator which will return the correct position and
 * velocity when running in simulation.
 */
template <typename T>
class CheaterPositionVelocityEstimator : public GenericEstimator<T>
{
public:
    virtual void run();
    virtual void setup() {}
};

#endif // ZQ_HUMANOID_POSITIONVELOCITYESTIMATOR_H
