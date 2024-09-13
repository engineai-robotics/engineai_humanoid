//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_QUADRUPED_H
#define ZQ_HUMANOID_QUADRUPED_H

#include <vector>
#include "ActuatorModel.h"
#include "FloatingBaseModel.h"
#include "SpatialInertia.h"

#include <eigen3/Eigen/StdVector>

/*!
 * Basic parameters for a biped-shaped robot
 */
namespace engineai_biped
{
    constexpr size_t num_act_joint = 6;
    constexpr size_t dim_config = 12;
    constexpr size_t num_leg = 2;
    constexpr size_t num_leg_joint = 3;
} // namespace engineai_biped

/*!
 * Link indices for engineai_biped-shaped robots
 */
namespace linkID
{
    constexpr size_t FR = 9;  // Front Right Foot
    constexpr size_t FL = 11; // Front Left Foot

    constexpr size_t FR_abd = 2; // Front Right Abduction
    constexpr size_t FL_abd = 0; // Front Left Abduction
    constexpr size_t HR_abd = 3; // Hind Right Abduction
    constexpr size_t HL_abd = 1; // Hind Left Abduction
} // namespace linkID

using std::vector;

/*!
 * Representation of a humanoid_biped robot's physical properties.
 *
 * When viewed from the back, the humanoid_biped's legs and joints sequences are:
 *
 * UP
 *   6     0
 *   7     1
 *   8     2   RIGHT
 *   9     3
 * 11 10  4  5 // ankle joint of close chain
 * BOTTOM
 *
 */
template <typename T>
class RobotConstructor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RobotType _robotType;
    T _bodyLength, _bodyWidth, _bodyHeight, _bodyMass;
    T _abadGearRatio, _hipGearRatio, _kneeGearRatio;
    T _abadLinkLength, _hipLinkLength, _kneeLinkLength, _kneeLinkY_offset, _maxLegLength;
    T _motorKT, _motorR, _batteryV;
    T _motorTauMax;
    T _jointDamping, _jointDryFriction;
    SpatialInertia<T> _abadInertia, _hipInertia, _kneeInertia, _abadRotorInertia,
        _hipRotorInertia, _kneeRotorInertia, _bodyInertia;
    Vec3<T> _abadLocation, _abadRotorLocation, _hipLocation, _hipRotorLocation,
        _kneeLocation, _kneeRotorLocation;
    FloatingBaseModel<T> buildModel();
    bool buildModel(FloatingBaseModel<T> &model);
    std::vector<ActuatorModel<T>> buildActuatorModels();

    /*!
     * Get if the i-th leg is on the left (+) or right (-) of the robot.
     * @param leg : the leg index
     * @return The side sign (-1 for right legs, +1 for left legs)
     */
    static T getSideSign(int leg)
    {
        const T sideSigns[4] = {-1, 1, -1, 1};
        assert(leg >= 0 && leg < 4);
        return sideSigns[leg];
    }

    /*!
     * Get location of the hip for the given leg in robot frame
     * @param leg : the leg index
     */
    Vec3<T> getHipLocation(int leg)
    {
        assert(leg >= 0 && leg < 2);
        Vec3<T> pHip(_abadLocation(0),
                     (leg == 1) ? _abadLocation(1) : -_abadLocation(1),
                     _abadLocation(2));
        return pHip;
    }
};

template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2> &v, int legID);

#endif // ZQ_HUMANOID_QUADRUPED_H
