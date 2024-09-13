//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_FOOTSWINGTRAJECTORY_H
#define ZQ_HUMANOID_FOOTSWINGTRAJECTORY_H

#include "../cppTypes.h"

/*!
 * A foot swing trajectory for a single foot
 */
template <typename T>
class FootSwingTrajectory
{
public:
    /*!
     * Construct a new foot swing trajectory with everything set to zero
     */
    FootSwingTrajectory()
    {
        _p0.setZero();
        _pf.setZero();
        _p.setZero();
        _v.setZero();
        _a.setZero();
        _height = 0;
    }

    /*!
     * Set the starting location of the foot
     * @param p0 : the initial foot position
     */
    void setInitialPosition(Vec3<T> p0)
    {
        _p0 = p0;
    }

    /*!
     * Set the desired final position of the foot
     * @param pf : the final foot posiiton
     */
    void setFinalPosition(Vec3<T> pf)
    {
        _pf = pf;
    }

    /*!
     * Set the maximum height of the swing
     * @param h : the maximum height of the swing, achieved halfway through the swing
     */
    void setHeight(T h)
    {
        _height = h;
    }

    void computeSwingTrajectoryBezier(T phase, T swingTime);

    /*!
     * Get the foot position at the current point along the swing
     * @return : the foot position
     */
    Vec3<T> getPosition()
    {
        return _p;
    }

    Vec3<T> getP0()
    {
        return _p0;
    }

    /*!
     * Get the foot velocity at the current point along the swing
     * @return : the foot velocity
     */
    Vec3<T> getVelocity()
    {
        return _v;
    }

    /*!
     * Get the foot acceleration at the current point along the swing
     * @return : the foot acceleration
     */
    Vec3<T> getAcceleration()
    {
        return _a;
    }

private:
    Vec3<T> _p0, _pf, _p, _v, _a;
    T _height;
};

#endif // ZQ_HUMANOID_FOOTSWINGTRAJECTORY_H
