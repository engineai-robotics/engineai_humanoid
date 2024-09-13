//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_SDUOG_H
#define ZQ_HUMANOID_SDUOG_H

#include "FloatingBaseModel.h"
#include "RobotConstructor.h"

/*!
 * Generate a robot model of ZQSA01 / ZQSA01-P
 */

template <typename T>
RobotConstructor<T> buildEngineAIRobot()
{
    RobotConstructor<T> engineai_biped;
    engineai_biped._robotType = RobotType::ZQ_Biped_SA01P;

    engineai_biped._bodyMass = 15.0;
    engineai_biped._bodyLength = 0.08 * 2; // 0.2055 * 2;
    engineai_biped._bodyWidth = 0.075 * 2; // 0.05 *2;
    engineai_biped._bodyHeight = 0.15 * 2;
    engineai_biped._abadGearRatio = 6;
    engineai_biped._hipGearRatio = 6;
    engineai_biped._kneeGearRatio = 6;
    engineai_biped._abadLinkLength = 0;    // 0.0576;//
    engineai_biped._hipLinkLength = 0.429; // 0.209;
    // engineai_biped._hipLinkLength = 0.329;//0.209;
    engineai_biped._kneeLinkY_offset = 0;  // 0.004;
    engineai_biped._kneeLinkLength = 0.37; // 0.195;
    engineai_biped._maxLegLength = 0.76;

    engineai_biped._motorTauMax = 20.f; // 3.f;
    engineai_biped._batteryV = 72;
    engineai_biped._motorKT = .05; // this is flux linkage * pole pairs
    engineai_biped._motorR = 0.173;
    engineai_biped._jointDamping = .01;
    engineai_biped._jointDryFriction = .2;

    // rotor inertia if the rotor is oriented so it spins around the z-axis
    Mat3<T> rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 133.72, 0, 0, 0, 133.72, 0, 0, 0, 257.61;
    rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

    Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
    Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
    Mat3<T> rotorRotationalInertiaX =
        RY * rotorRotationalInertiaZ * RY.transpose();
    Mat3<T> rotorRotationalInertiaY =
        RX * rotorRotationalInertiaZ * RX.transpose();

    // spatial inertias
    Mat3<T> abadRotationalInertia;
    abadRotationalInertia << 1842, -0, 0, -0, 2341, 0, 0, 0, 3266;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vec3<T> abadCOM(0, 0, 0); // LEFT  ?!??????????????????
    SpatialInertia<T> abadInertia(1.5, abadCOM, abadRotationalInertia);

    Mat3<T> hipRotationalInertia;
    hipRotationalInertia << 34081, 43, 1284, 43, 37772, -1174, 1284, -1174, 10326;
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    Vec3<T> hipCOM(0, 0.0, -0.143);
    SpatialInertia<T> hipInertia(3.9, hipCOM, hipRotationalInertia);

    Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 34519, -183, -593, -183, 36273, 786, -593, 786, 4851;
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vec3<T> kneeCOM(0, 0, -0.16);
    SpatialInertia<T> kneeInertia(3.09, kneeCOM, kneeRotationalInertia);

    Vec3<T> rotorCOM(0, 0, 0);
    SpatialInertia<T> rotorInertiaX(0.22, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<T> rotorInertiaY(0.22, rotorCOM, rotorRotationalInertiaY);

    Mat3<T> bodyRotationalInertia;
    bodyRotationalInertia << 142832, 0, 0, 0, 88957, 0, 0, 0, 87302;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    //    Vec3<T> bodyCOM(-0.025, 0, 0.13619966);
    Vec3<T> bodyCOM(-0, 0, 0.);
    SpatialInertia<T> bodyInertia(engineai_biped._bodyMass, bodyCOM, bodyRotationalInertia);

    engineai_biped._abadInertia = abadInertia;
    engineai_biped._hipInertia = hipInertia;
    engineai_biped._kneeInertia = kneeInertia;
    engineai_biped._abadRotorInertia = rotorInertiaX;
    engineai_biped._hipRotorInertia = rotorInertiaY;
    engineai_biped._kneeRotorInertia = rotorInertiaY;
    engineai_biped._bodyInertia = bodyInertia;

    // locations
    //  engineai_biped._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
    engineai_biped._abadRotorLocation = Vec3<T>(engineai_biped._bodyLength, engineai_biped._bodyWidth, 0) * 0.5;
    //    engineai_biped._abadLocation =Vec3<T>(engineai_biped._bodyLength, engineai_biped._bodyWidth, 0) * 0.5;
    engineai_biped._abadLocation = Vec3<T>(0, engineai_biped._bodyWidth, 0) * 0.5;
    engineai_biped._hipLocation = Vec3<T>(0, engineai_biped._abadLinkLength, 0);
    engineai_biped._hipRotorLocation = Vec3<T>(0, 0.0, 0);
    engineai_biped._kneeLocation = Vec3<T>(0, 0, -engineai_biped._hipLinkLength);
    engineai_biped._kneeRotorLocation = Vec3<T>(0, 0, 0);

    return engineai_biped;
}

#endif // ZQ_HUMANOID_SDUOG_H
