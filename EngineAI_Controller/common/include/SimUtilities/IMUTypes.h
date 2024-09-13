//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_IMUTYPES_H
#define ZQ_HUMANOID_IMUTYPES_H

#include "../cppTypes.h"

struct VectorNavData
{
    Vec3<float> accelerometer;
    Vec3<float> gyro;
    Quat<float> quat;
    // todo is there status for the vectornav?
};

/*!
 * "Cheater" state sent to the robot from simulator
 */
template <typename T>
struct CheaterState
{
    Quat<T> orientation;
    Vec3<T> position;
    Vec3<T> omegaBody;
    Vec3<T> vBody;
    Vec3<T> acceleration;
};

#endif // ZQ_HUMANOID_IMUTYPES_H
