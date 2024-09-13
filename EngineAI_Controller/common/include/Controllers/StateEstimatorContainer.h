//
// Created by engineai on 2024/07/03.
//
/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */
#ifndef ZQ_HUMANOID_STATEESTIMATORCONTAINER_H
#define ZQ_HUMANOID_STATEESTIMATORCONTAINER_H

#include "../ControlParameters/RobotParameters.h"
#include "LegController.h"
#include "../SimUtilities/IMUTypes.h"
#include "../SimUtilities/VisualizationData.h"
#include "../../../lcm-types/cpp/state_estimator_lcmt.hpp"

// #include "ControlParameters/RobotParameters.h"
// #include "Controllers/LegController.h"
// #include "SimUtilities/IMUTypes.h"
// #include "SimUtilities/VisualizationData.h"
// #include "state_estimator_lcmt.hpp"

/*!
 * Result of state estimation
 */
template <typename T>
struct StateEstimate
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec2<T> contactEstimate;
    Vec3<T> position;
    Vec3<T> vBody;
    Quat<T> orientation;
    Vec3<T> omegaBody;
    RotMat<T> rBody;
    Vec3<T> rpy;
    Quat<T> ori_quat;

    Vec3<T> omegaWorld;
    Vec3<T> vWorld;
    Vec3<T> aBody, aWorld;
    Vec3<T> rRemoterVelociy;
    Vec3<T> GPS_Position;

    void setLcm(state_estimator_lcmt &lcm_data)
    {
        static int32_t state_estimator_containor_count = 0;
        state_estimator_containor_count++;
        for (int i = 0; i < 3; i++)
        {
            lcm_data.p[i] = position[i];
            lcm_data.vWorld[i] = vWorld[i];
            lcm_data.vBody[i] = vBody[i];
            lcm_data.rpy[i] = rpy[i];
            lcm_data.omegaBody[i] = omegaBody[i];
            lcm_data.omegaWorld[i] = omegaWorld[i];
            lcm_data.vRemoter[i] = rRemoterVelociy[i];
            lcm_data.aBody[i] = aBody[i];
            lcm_data.aWorld[i] = aWorld[i];
            // if(state_estimator_containor_count%1000==0){
            //     std::cout << "\033[1;32m legTorque="<<legTorque<<" \033[0m \n";
            // }
        }

        for (int i = 0; i < 4; i++)
        {
            lcm_data.quat[i] = orientation[i];
        }
    }
};

/*!
 * Inputs for state estimation.
 * If robot code needs to inform the state estimator of something,
 * it should be added here. (You should also a setter method to
 * StateEstimatorContainer)
 */
template <typename T>
struct StateEstimatorData
{
    StateEstimate<T> *result; // where to write the output to
    VectorNavData *vectorNavData;
    CheaterState<double> *cheaterState;
    LegControllerData<T> *legControllerData;
    Vec2<T> *contactPhase;
    RobotControlParameters *parameters;
};

/*!
 * All Estimators should inherit from this class
 */
template <typename T>
class GenericEstimator
{
public:
    virtual void run() = 0;
    virtual void setup() = 0;

    void setData(StateEstimatorData<T> data) { _stateEstimatorData = data; }

    virtual ~GenericEstimator() = default;
    StateEstimatorData<T> _stateEstimatorData;
};

/*!
 * Main State Estimator Class
 * Contains all GenericEstimators, and can run them
 * Also updates visualizations
 */
template <typename T>
class StateEstimatorContainer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * Construct a new state estimator container
     */
    StateEstimatorContainer(CheaterState<double> *cheaterState,
                            VectorNavData *vectorNavData,
                            LegControllerData<T> *legControllerData,
                            StateEstimate<T> *stateEstimate,
                            RobotControlParameters *parameters)
    {
        _data.cheaterState = cheaterState;
        _data.vectorNavData = vectorNavData;
        _data.legControllerData = legControllerData;
        _data.result = stateEstimate;
        _phase = Vec2<T>::Zero();
        _data.contactPhase = &_phase;
        _data.parameters = parameters;
    }

    /*!
     * Run all estimators
     */
    void run(EngineAIRobotVisualization *visualization = nullptr)
    {
        for (auto estimator : _estimators)
        {
            estimator->run();
        }
        if (visualization)
        {
            visualization->quat = _data.result->orientation.template cast<float>();
            visualization->p = _data.result->position.template cast<float>();
            // todo contact!
        }
    }

    /*!
     * Get the result
     */
    const StateEstimate<T> &getResult() { return *_data.result; }
    void setRemoterVelocityResult(Vec3<T> rs)
    {
        _data.result->rRemoterVelociy = rs;
        //      printf("remoterVelocity:%.2f\t%.2f\t%.2f\n",rs(0),rs(1),rs(2));
    }
    StateEstimate<T> *getResultHandle() { return _data.result; }

    /*!
     * Set the contact phase
     */
    void setContactPhase(Vec2<T> &phase)
    {
        *_data.contactPhase = phase;
    }

    /*!
     * Add an estimator of the given type
     * @tparam EstimatorToAdd
     */
    template <typename EstimatorToAdd>
    void addEstimator()
    {
        auto *estimator = new EstimatorToAdd();
        estimator->setData(_data);
        estimator->setup();
        _estimators.push_back(estimator);
    }

    /*!
     * Remove all estimators of a given type
     * @tparam EstimatorToRemove
     */
    template <typename EstimatorToRemove>
    void removeEstimator()
    {
        int nRemoved = 0;
        _estimators.erase(
            std::remove_if(_estimators.begin(), _estimators.end(),
                           [&nRemoved](GenericEstimator<T> *e)
                           {
                               if (dynamic_cast<EstimatorToRemove *>(e))
                               {
                                   delete e;
                                   nRemoved++;
                                   return true;
                               }
                               else
                               {
                                   return false;
                               }
                           }),
            _estimators.end());
    }

    /*!
     * Remove all estimators
     */
    void removeAllEstimators()
    {
        for (auto estimator : _estimators)
        {
            delete estimator;
        }
        _estimators.clear();
    }

    ~StateEstimatorContainer()
    {
        for (auto estimator : _estimators)
        {
            delete estimator;
        }
    }
    StateEstimatorData<T> GetEstimatorData()
    {
        return _data;
    }

private:
    StateEstimatorData<T> _data;
    std::vector<GenericEstimator<T> *> _estimators;
    Vec2<T> _phase;
};

#endif // ZQ_HUMANOID_STATEESTIMATORCONTAINER_H
