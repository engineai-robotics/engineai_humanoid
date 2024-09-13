//
// Created by engineai on 2024/07/03.
//
/*! @file RobotConstructor.cpp
 *  @brief Data structure containing parameters for humanoid_biped robot
 *
 *  This file contains the RobotConstructor class.  This stores all the parameters for
 * a humanoid_biped robot.  There are utility functions to generate RobotConstructor objects
 * for SA01 and SA01-P. There is a buildModel() method which can be used to create a floating-base dynamics model of the humanoid_biped.
 */

#include "Dynamics/RobotConstructor.h"
#include "Dynamics/spatial.h"
#include "Math/orientation_tools.h"

using namespace ori;
using namespace spatial;

/*!
 * Build a FloatingBaseModel of the humanoid_biped
 */
template <typename T>
bool RobotConstructor<T>::buildModel(FloatingBaseModel<T> &model)
{
    // we assume the engineai_biped's body (not including rotors) can be modeled as a
    // uniformly distributed box.
    Vec3<T> bodyDims(_bodyLength, _bodyWidth, _bodyHeight);
    // model.addBase(_bodyMass, Vec3<T>(0,0,0), rotInertiaOfBox(_bodyMass,
    // bodyDims));
    model.addBase(_bodyInertia);
    model.addGroundContactBoxPoints(5, bodyDims);

    const int baseID = 5;
    int bodyID = baseID;
    T sideSign = -1;

    Mat3<T> I3 = Mat3<T>::Identity();

    // loop over 4 legs
    //    for (int legID = 0; legID < 4; legID++) {
    for (int legID = 0; legID < 2; legID++)
    {

        bodyID++;
        Mat6<T> xtreeAbad = createSXform(I3, withLegSigns<T>(_abadLocation, legID)); // abad位置,加入到tree中
        Mat6<T> xtreeAbadRotor =
            createSXform(I3, withLegSigns<T>(_abadRotorLocation, legID));

        if (sideSign < 0)
        {
            model.addBody(_abadInertia.flipAlongAxis(CoordinateAxis::Y),
                          _abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                          _abadGearRatio, baseID, JointType::Revolute,
                          CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);
        }
        else
        {
            model.addBody(_abadInertia, _abadRotorInertia, _abadGearRatio, baseID,
                          JointType::Revolute, CoordinateAxis::X, xtreeAbad,
                          xtreeAbadRotor);
        }

        // Hip Joint
        bodyID++;
        Mat6<T> xtreeHip =
            createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
                         withLegSigns<T>(_hipLocation, legID));
        Mat6<T> xtreeHipRotor =
            createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
                         withLegSigns<T>(_hipRotorLocation, legID));
        if (sideSign < 0)
        {
            model.addBody(_hipInertia.flipAlongAxis(CoordinateAxis::Y),
                          _hipRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                          _hipGearRatio, bodyID - 1, JointType::Revolute,
                          CoordinateAxis::Y, xtreeHip, xtreeHipRotor);
        }
        else
        {
            model.addBody(_hipInertia, _hipRotorInertia, _hipGearRatio, bodyID - 1,
                          JointType::Revolute, CoordinateAxis::Y, xtreeHip,
                          xtreeHipRotor);
        }

        // add knee ground contact point
        model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_hipLinkLength));

        // Knee Joint
        bodyID++;
        Mat6<T> xtreeKnee = createSXform(I3, _kneeLocation);
        Mat6<T> xtreeKneeRotor = createSXform(I3, _kneeRotorLocation);
        if (sideSign < 0)
        {
            model.addBody(_kneeInertia.flipAlongAxis(CoordinateAxis::Y),
                          _kneeRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                          _kneeGearRatio, bodyID - 1, JointType::Revolute,
                          CoordinateAxis::Y, xtreeKnee, xtreeKneeRotor);

            model.addGroundContactPoint(bodyID, Vec3<T>(0, _kneeLinkY_offset, -_kneeLinkLength), true);
        }
        else
        {
            model.addBody(_kneeInertia, _kneeRotorInertia, _kneeGearRatio, bodyID - 1,
                          JointType::Revolute, CoordinateAxis::Y, xtreeKnee,
                          xtreeKneeRotor);

            model.addGroundContactPoint(bodyID, Vec3<T>(0, -_kneeLinkY_offset, -_kneeLinkLength), true);
        }

        // add foot
        // model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_kneeLinkLength), true);

        sideSign *= -1;
    }

    Vec3<T> g(0, 0, -9.81);
    model.setGravity(g);

    return true;
}

/*!
 * Build a FloatingBaseModel of the humanoid_biped
 */
template <typename T>
FloatingBaseModel<T> RobotConstructor<T>::buildModel()
{
    FloatingBaseModel<T> model;
    buildModel(model);
    return model;
}

/*!
 * Flip signs of elements of a vector V depending on which leg it belongs to
 */
template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2> &v, int legID)
{
    static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                  "Must have 3x1 matrix");
    switch (legID)
    {
    case 0:
        return Vec3<T>(v[0], -v[1], v[2]);
    case 1:
        return Vec3<T>(v[0], v[1], v[2]);
    default:
        throw std::runtime_error("Invalid leg id!");
    }
}

/*!
 * Build actuator models for a leg
 */
template <typename T>
std::vector<ActuatorModel<T>> RobotConstructor<T>::buildActuatorModels()
{
    std::vector<ActuatorModel<T>> models;
    models.emplace_back(_abadGearRatio, _motorKT, _motorR, _batteryV,
                        _jointDamping, _jointDryFriction, _motorTauMax);
    models.emplace_back(_hipGearRatio, _motorKT, _motorR, _batteryV,
                        _jointDamping, _jointDryFriction, _motorTauMax);
    models.emplace_back(_kneeGearRatio, _motorKT, _motorR, _batteryV,
                        _jointDamping, _jointDryFriction, _motorTauMax);
    return models;
}

template class RobotConstructor<double>;
template class RobotConstructor<float>;
