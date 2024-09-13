//
// Created by engineai on 2024/07/03.
//
/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control algorithm and hardware/simulation
 */

#include <unistd.h>
#include <fstream>

#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "../../common/include/Dynamics/EngineAIBiped.h"
#include "Utilities/Utilities_print.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "../include/RobotRunner.h"
#include <chrono>

// #define RLEARNING_CONTROL

RobotRunner::RobotRunner(RobotController *robot_ctrl,
                         PeriodicTaskManager *manager,
                         float period, std::string name) : PeriodicTask(manager, period, name),
                                                           _lcm(getLcmUrl(255))
{
    _robot_ctrl = robot_ctrl;

    motor_enable_flag = false;
    motor_disable_flag = false;

    // outFile("humanoid_datalog.csv", std::ios::trunc);
    data_log_file.open("tau_delay_and_thread_RT.csv", std::ios::trunc);
}

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::init()
{
    printf("[RobotRunner] initialize\n");

    _humanoid_biped = buildEngineAIRobot<float>();

    // Initialize the model and robot data
    _model = _humanoid_biped.buildModel();

    // Always initialize the leg controller and state entimator
    _legController = new LegController<float>(_humanoid_biped, controlParameters);

    _legController->motor_enable_flag = &motor_enable_flag;
    _legController->motor_disable_flag = &motor_disable_flag;

    _stateEstimator = new StateEstimatorContainer<float>(
        cheaterState, vectorNavData, _legController->datas,
        &_stateEstimate, controlParameters);

    initializeStateEstimator(false);

    memset(&rc_control, 0, sizeof(rc_control_settings));

    // Initialize the DesiredStateCommand object
    _desiredStateCommand =
        new DesiredStateCommand<float>(driverCommand,
                                       &rc_control,
                                       controlParameters,
                                       &_stateEstimate,
                                       controlParameters->controller_dt);

    // Controller initializations
    _robot_ctrl->_model = &_model;
    _robot_ctrl->_humanoid_biped = &_humanoid_biped;
    _robot_ctrl->_legController = _legController;
    _robot_ctrl->_stateEstimator = _stateEstimator;
    _robot_ctrl->_stateEstimate = &_stateEstimate;
    _robot_ctrl->_visualizationData = visualizationData;
    _robot_ctrl->_robotType = robotType;
    _robot_ctrl->_driverCommand = driverCommand;
    _robot_ctrl->_controlParameters = controlParameters;
    _robot_ctrl->_desiredStateCommand = _desiredStateCommand;

    _robot_ctrl->initializeController();

    _ankle_joint_mapping = new Decouple();
}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 */
void RobotRunner::run()
{
    static int times = 0;
    times++;

    // Run the state estimator step
    auto se_start_time = std::chrono::high_resolution_clock::now();
    _stateEstimator->run();

    auto se_current_time = std::chrono::high_resolution_clock::now();

    state_estimate_time = std::chrono::duration_cast<std::chrono::nanoseconds>(se_current_time - se_start_time).count() / 1e6;

    visualizationData->clear();

    // Update the data from the robot
    setupStep();

    static int count_ini(0);
    ++count_ini;
    if (count_ini < 10)
    {
        _legController->setEnabled(false);
    }
    else
    {

        auto rl_start_time = std::chrono::high_resolution_clock::now();
        _robot_ctrl->runController();

        auto rl_current_time = std::chrono::high_resolution_clock::now();

        rl_inference_time = std::chrono::duration_cast<std::chrono::nanoseconds>(rl_current_time - rl_start_time).count() / 1e6;
    }

    // Visualization (will make this into a separate function later)

    // Sets the leg controller commands for the robot appropriate commands
    finalizeStep();
}

/*!
 * Before running user code, setup the leg control and estimators
 */
void RobotRunner::setupStep()
{
    // Update the leg data
    if (robotType == RobotType::ZQ_Biped_SA01 || robotType == RobotType::ZQ_Biped_SA01P)
    {
        _legController->updateData(jointDatas);
    }
    else
    {
        assert(false);
    }

    // Setup the leg controller for a new iteration
    _legController->zeroCommand();
    _legController->setEnabled(true);

    get_rc_control_settings(&rc_control);

    // todo safety checks, sanity checks, etc...
}

/*!
 * After the user code, send leg commands, update state estimate, and publish debug data
 */
auto start_time = std::chrono::high_resolution_clock::now();
auto pre_time = std::chrono::high_resolution_clock::now();
void RobotRunner::finalizeStep()
{
    if (robotType == RobotType::ZQ_Biped_SA01 || robotType == RobotType::ZQ_Biped_SA01P)
    {
        if (controlParameters->use_joint_test_RL)
        {
            JointControlTestwithYAML();
            // CloseChainMappingTest();
        }
        else
        {
            _legController->updateCommand(jointCommand);
        }

        auto current_time = std::chrono::high_resolution_clock::now();

        float delta_time = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - pre_time).count() / 1e6;
        float total_time = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - start_time).count() / 1e6;

        pre_time = current_time;

        // if (data_log_file.is_open())
        // {
        //     if (first_run)
        //     {
        //         first_run = false;
        //         data_log_file << "state_estimate_time,rl_inference_time" << ","
        //                       << "total_time,delta_time_ms" << ","
        //                       << "cmd_tau_0,cmd_tau_1,cmd_tau_2,cmd_tau_3,cmd_tau_4,cmd_tau_5" << ","
        //                       << "cmd_tau_6,cmd_tau_7,cmd_tau_8,cmd_tau_9,cmd_tau_10,cmd_tau_11" << ","
        //                       << "state_tau_0,state_tau_1,state_tau_2,state_tau_3,state_tau_4,state_tau_5" << ","
        //                       << "state_tau_6,state_tau_7,state_tau_8,state_tau_9,state_tau_10,state_tau_11" << "\n";
        //     }
        //     else
        //     {
        //         data_log_file << state_estimate_time << "," << rl_inference_time << ","
        //                       << total_time << "," << delta_time << ","
        //                       << jointCommand->tau_ff[0] << "," << jointCommand->tau_ff[1] << "," << jointCommand->tau_ff[2] << ","
        //                       << jointCommand->tau_ff[3] << "," << jointCommand->tau_ff[4] << "," << jointCommand->tau_ff[5] << ","
        //                       << jointCommand->tau_ff[6] << "," << jointCommand->tau_ff[7] << "," << jointCommand->tau_ff[8] << ","
        //                       << jointCommand->tau_ff[9] << "," << jointCommand->tau_ff[10] << "," << jointCommand->tau_ff[11] << ","
        //                       << jointDatas->tau[0] << "," << jointDatas->tau[1] << "," << jointDatas->tau[2] << ","
        //                       << jointDatas->tau[3] << "," << jointDatas->tau[4] << "," << jointDatas->tau[5] << ","
        //                       << jointDatas->tau[6] << "," << jointDatas->tau[7] << "," << jointDatas->tau[8] << ","
        //                       << jointDatas->tau[9] << "," << jointDatas->tau[10] << "," << jointDatas->tau[11] << "\n";
        //     }
        // }
        // else
        //     std::cerr << "Unable to open file\n";
    }
    else
    {
        assert(false);
    }

    _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
    _stateEstimate.setLcm(state_estimator_lcm);

    _lcm.publish("leg_control_command", &leg_control_command_lcm);
    _lcm.publish("leg_control_data", &leg_control_data_lcm);
    _lcm.publish("state_estimator", &state_estimator_lcm);

    _iterations++;
}

void RobotRunner::CloseChainMappingTest()
{
    _legController->zeroCommand();
    _legController->setEnabled(true);

    // tau mapping test
    // step 1: read the current joint state
    Eigen::VectorXd q = Eigen::MatrixXd::Zero(12, 1);
    Eigen::VectorXd vel = Eigen::MatrixXd::Zero(12, 1);
    Eigen::VectorXd tau = Eigen::MatrixXd::Zero(12, 1);

    q[4] = jointDatas->q[4];
    q[5] = jointDatas->q[5];
    q[10] = jointDatas->q[10];
    q[11] = jointDatas->q[11];

    vel[4] = jointDatas->qd[4];
    vel[5] = jointDatas->qd[5];
    vel[10] = jointDatas->qd[10];
    vel[11] = jointDatas->qd[11];

    // step 2: forward mapping to get the ankle ori
    _ankle_joint_mapping->getForwardQVT(q, vel, tau); //

    // step 3: ankle ori command
    float amp = 0.4;
    float period = controlParameters->sin_wave_period; // second
    float cur_time = _iterations * 0.002;

    float right_pitch_cmd = amp * sin(2 * M_PI / period * cur_time);
    float right_roll_cmd = 0; //-amp * sin(2 * M_PI / period * cur_time);
    float left_pitch_cmd = amp * sin(2 * M_PI / period * cur_time);
    float left_roll_cmd = 0; //-amp * sin(2 * M_PI / period * cur_time);

    // step 4: ankle PD control
    tau[4] = controlParameters->leg_0_kp[1] * (left_pitch_cmd - q[4]) + controlParameters->leg_0_kd[1] * (0 - vel[4]);
    tau[5] = controlParameters->leg_0_kp[2] * (left_roll_cmd - q[5]) + controlParameters->leg_0_kd[2] * (0 - vel[5]);
    tau[10] = controlParameters->leg_2_kp[1] * (right_pitch_cmd - q[10]) + controlParameters->leg_2_kd[1] * (0 - vel[10]);
    tau[11] = controlParameters->leg_2_kp[2] * (right_roll_cmd - q[11]) + controlParameters->leg_2_kd[2] * (0 - vel[11]);

    // step 5: tau mapping from ankle joint to motor
    _ankle_joint_mapping->getDecoupleQVT(q, vel, tau); // tau: get motor torques

    for (int i = 0; i < 12; i++)
    {
        jointCommand->q_des[i] = 0.0;
        jointCommand->qd_des[i] = 0.0;
        jointCommand->tau_ff[i] = 0.0;
        jointCommand->kp[i] = 0.0;
        jointCommand->kd[i] = 0.0;
    }

    jointCommand->tau_ff[4] = tau[4];
    jointCommand->tau_ff[5] = tau[5];
    jointCommand->tau_ff[10] = tau[10];
    jointCommand->tau_ff[11] = tau[11];

    printf("motor_tau_cmd: %f, %f, %f, %f\n", tau[4], tau[5], tau[10], tau[11]);
}
void RobotRunner::JointControlTestwithYAML()
{
    _legController->zeroCommand();
    _legController->setEnabled(true);

    for (int i = 0; i < 12; i++)
    {
        if (i < 3)
        {
            jointCommand->q_des[i] = controlParameters->leg_0_qdes[i];
            jointCommand->qd_des[i] = controlParameters->leg_0_qddes[i];
            jointCommand->tau_ff[i] = controlParameters->leg_0_tauff[i];
            jointCommand->kp[i] = controlParameters->leg_0_kp[i];
            jointCommand->kd[i] = controlParameters->leg_0_kd[i];
        }
        else if (i < 6)
        {
            jointCommand->q_des[i] = controlParameters->leg_1_qdes[i - 3];
            jointCommand->qd_des[i] = controlParameters->leg_1_qddes[i - 3];
            jointCommand->tau_ff[i] = controlParameters->leg_1_tauff[i - 3];
            jointCommand->kp[i] = controlParameters->leg_1_kp[i - 3];
            jointCommand->kd[i] = controlParameters->leg_1_kd[i - 3];
        }
        else if (i < 9)
        {
            jointCommand->q_des[i] = controlParameters->leg_2_qdes[i - 6];
            jointCommand->qd_des[i] = controlParameters->leg_2_qddes[i - 6];
            jointCommand->tau_ff[i] = controlParameters->leg_2_tauff[i - 6];
            jointCommand->kp[i] = controlParameters->leg_2_kp[i - 6];
            jointCommand->kd[i] = controlParameters->leg_2_kd[i - 6];
        }
        else
        {
            jointCommand->q_des[i] = controlParameters->leg_3_qdes[i - 9];
            jointCommand->qd_des[i] = controlParameters->leg_3_qddes[i - 9];
            jointCommand->tau_ff[i] = controlParameters->leg_3_tauff[i - 9];
            jointCommand->kp[i] = controlParameters->leg_3_kp[i - 9];
            jointCommand->kd[i] = controlParameters->leg_3_kd[i - 9];
        }
    }

    float amp = 0.3;
    float period = controlParameters->sin_wave_period; // second
    float cur_time = _iterations * 0.002;

    double target_pos = amp * sin(2 * M_PI / period * cur_time);

    for (int i = 0; i < 12; i++)
    {
        jointCommand->q_des[i] = target_pos;
    }

    if (target_pos < 0.0)
        target_pos = 0.0;
    jointCommand->q_des[3] = target_pos;
    jointCommand->q_des[9] = target_pos;
}

/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator(bool cheaterMode)
{
    _stateEstimator->removeAllEstimators();
    _stateEstimator->addEstimator<ContactEstimator<float>>();
    Vec2<float> contactDefault;
    contactDefault << 0.5, 0.5;
    _stateEstimator->setContactPhase(contactDefault);
    if (cheaterMode)
    {
        _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
        _stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();
    }
    else
    {
        _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
        _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
    }
}

RobotRunner::~RobotRunner()
{
    data_log_file.close();
    delete _legController;
    delete _stateEstimator;
}

void RobotRunner::cleanup() {}
