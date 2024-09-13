//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_CONTACTESTIMATOR_H
#define ZQ_HUMANOID_CONTACTESTIMATOR_H

#include "StateEstimatorContainer.h"

/*!
 * A "passthrough" contact estimator which returns the expected contact state
 */
template <typename T>
class ContactEstimator : public GenericEstimator<T>
{
public:
    /*!
     * Set the estimated contact by copying the exptected contact state into the
     * estimated contact state
     */
    virtual void run()
    {
        this->_stateEstimatorData.result->contactEstimate =
            *this->_stateEstimatorData.contactPhase;
    }

    /*!
     * Set up the contact estimator
     */
    virtual void setup() {}
};
#endif // ZQ_HUMANOID_CONTACTESTIMATOR_H
