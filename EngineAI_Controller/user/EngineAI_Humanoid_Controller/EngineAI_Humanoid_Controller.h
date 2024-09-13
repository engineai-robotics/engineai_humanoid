//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_CONTROLLER_H
#define ZQ_HUMANOID_CONTROLLER_H

#include "../../robot/include/RobotController.h"
#include "../../common/include/Controllers/GaitScheduler.h"
#include "../../common/include/Controllers/ContactEstimator.h"
#include "FSM_States/ControlFSM.h"
#include "EngineAI_UserParameters.h"
#include <lcm/lcm.h>

class EngineAI_Humanoid_Controller : public RobotController
{
public:
    EngineAI_Humanoid_Controller();
    // EngineAI_Humanoid_Controller(const ros::NodeHandle& n);
    virtual ~EngineAI_Humanoid_Controller()
    {
    }

    virtual void initializeController();
    virtual void runController();
    virtual void updateVisualization() {}
    virtual ControlParameters *getUserControlParameters()
    {
        return &userParameters;
    }
    virtual void Estop() { _controlFSM->initialize(); }

protected:
    ControlFSM<float> *_controlFSM;
    // Gait Scheduler controls the nominal contact schedule for the feet
    GaitScheduler<float> *_gaitScheduler;
    EngineAI_UserParameters userParameters;
    lcm::LCM lcm_;
    lcm::LCM lcm_pub_jointstate;
};

#endif // ZQ_HUMANOID_CONTROLLER_H
