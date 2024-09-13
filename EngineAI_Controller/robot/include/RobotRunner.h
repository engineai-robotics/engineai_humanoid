//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_ROBOTRUNNER_H
#define ZQ_HUMANOID_ROBOTRUNNER_H

#include "../../common/include/ControlParameters/ControlParameterInterface.h"
#include "../../common/include/ControlParameters/RobotParameters.h"
#include "../../common/include/Controllers/StateEstimatorContainer.h"
#include "../../common/include/SimUtilities/IMUTypes.h"
#include "rt/rt_rc_interface.h"
#include "../../common/include/Controllers/ContactEstimator.h"
#include "../../common/include/Controllers/DesiredStateCommand.h"
#include "../../common/include/Controllers/LegController.h"
#include "../../common/include/Controllers/close_chain_mapping.h"
#include "../../common/include/Dynamics/RobotConstructor.h"

#include "../../common/include/SimUtilities/GamepadCommand.h"
#include "../../common/include/SimUtilities/VisualizationData.h"
#include "../../common/include/Utilities/PeriodicTask.h"
#include "../../lcm-types/cpp/state_estimator_lcmt.hpp"
#include "../include/RobotController.h"
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <fstream> // Include the file stream library
#include "motor.h"

class RobotRunner : public PeriodicTask
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RobotRunner(RobotController *, PeriodicTaskManager *, float, std::string);
    using PeriodicTask::PeriodicTask;
    void init() override;
    void run() override;
    void cleanup() override;

    // Initialize the state estimator with default no cheaterMode
    void initializeStateEstimator(bool cheaterMode = false);

    void JointControlTestwithYAML();
    void CloseChainMappingTest();

    virtual ~RobotRunner();

    RobotController *_robot_ctrl;

    GamepadCommand *driverCommand;
    RobotType robotType;
    VectorNavData *vectorNavData;
    CheaterState<double> *cheaterState;
    motor_data_t *jointDatas;
    motor_command_t *jointCommand;

    TiBoardCommand *tiBoardCommand;
    TiBoardData *tiBoardData;
    RobotControlParameters *controlParameters;
    VisualizationData *visualizationData;
    EngineAIRobotVisualization engineAIRobotVisualization;

    bool motor_enable_flag;
    bool motor_disable_flag;

private:
    float _ini_yaw;

    int iter = 0;

    void setupStep();
    void finalizeStep();

    RobotConstructor<float> _humanoid_biped;
    LegController<float> *_legController = nullptr;
    StateEstimate<float> _stateEstimate;
    StateEstimatorContainer<float> *_stateEstimator;
    bool _cheaterModeEnabled = false;
    DesiredStateCommand<float> *_desiredStateCommand;
    rc_control_settings rc_control;
    lcm::LCM _lcm;
    leg_control_command_lcmt leg_control_command_lcm;
    state_estimator_lcmt state_estimator_lcm;
    leg_control_data_lcmt leg_control_data_lcm;

    leg_control_data_lcmt leg_control_data_lcm_RL;

    FloatingBaseModel<float> _model;
    u64 _iterations = 0;
    int rl_count = 0;

    std::thread _RLearningLcmThread;
    volatile bool _RLearningLcmQuit = false;
    int cycle_cnt_ = 0;
    Decouple *_ankle_joint_mapping = nullptr;

    bool enable_tau_mapping_RL = false; // use keyboard remotely to enable tau mapping for ankle joints

    std::ofstream data_log_file;
    float right_ankle_vel_pitch = 0;
    float right_ankle_vel_roll = 0;
    float left_ankle_vel_pitch = 0;
    float left_ankle_vel_roll = 0;

    float right_ankle_tau_cmd_pitch = 0;
    float right_ankle_tau_cmd_roll = 0;
    float left_ankle_tau_cmd_pitch = 0;
    float left_ankle_tau_cmd_roll = 0;
    bool first_run = true;

    float rl_inference_time;
    float state_estimate_time;
};

#endif // ZQ_HUMANOID_ROBOTRUNNER_H
