//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_ROBOTCONTROLLER_H
#define ZQ_HUMANOID_ROBOTCONTROLLER_H

#include "common/include/Dynamics/FloatingBaseModel.h"
#include "common/include/Controllers/StateEstimatorContainer.h"
#include "common/include/Controllers/DesiredStateCommand.h"
#include "common/include/Controllers/LegController.h"
#include "common/include/SimUtilities/VisualizationData.h"
#include "common/include/SimUtilities/GamepadCommand.h"

/*!
 * Parent class of user robot controllers
 */
class RobotController
{
    friend class RobotRunner;

public:
    RobotController() {}
    virtual ~RobotController() {}

    virtual void initializeController() = 0;
    /**
     * Called once every control loop
     */
    virtual void runController() = 0;
    virtual void updateVisualization() = 0;
    virtual ControlParameters *getUserControlParameters() = 0;
    virtual void Estop() {}

protected:
    RobotConstructor<float> *_humanoid_biped = nullptr;
    FloatingBaseModel<float> *_model = nullptr;
    LegController<float> *_legController = nullptr;
    StateEstimatorContainer<float> *_stateEstimator = nullptr;
    StateEstimate<float> *_stateEstimate = nullptr;
    GamepadCommand *_driverCommand = nullptr;
    RobotControlParameters *_controlParameters = nullptr;
    DesiredStateCommand<float> *_desiredStateCommand = nullptr;

    VisualizationData *_visualizationData = nullptr;
    RobotType _robotType;
};

#endif // ZQ_HUMANOID_ROBOTCONTROLLER_H
