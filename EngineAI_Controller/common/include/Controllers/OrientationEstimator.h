//
// Created by engineai on 2024/07/03.
//
//
#ifndef ZQ_HUMANOID_ORIENTATIONESTIMATOR_H
#define ZQ_HUMANOID_ORIENTATIONESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"
#include "state_estimator_lcmt.hpp"
#include <lcm/lcm-cpp.hpp>
// #include "../config.h"
// using namespace webots;
/*!
 * "Cheater" estimator for orientation which always returns the correct value in simulation
 */
template <typename T>
class CheaterOrientationEstimator : public GenericEstimator<T>
{
public:
    CheaterOrientationEstimator() {};
    virtual void run();
    virtual void setup() {}
};

/*!
 * Estimator for the VectorNav IMU.  The VectorNav provides an orientation already and
 * we just return that.
 */
template <typename T>
class VectorNavOrientationEstimator : public GenericEstimator<T>
{
public:
    VectorNavOrientationEstimator();
    virtual void run();
    virtual void setup() {}

protected:
    bool _b_first_visit = true;
    Quat<T> _ori_ini_inv;
    struct timespec t_spec;
};

#endif // ZQ_HUMANOID_ORIENTATIONESTIMATOR_H
