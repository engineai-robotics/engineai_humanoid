//
// Created by engineai on 2024/07/03.
//
/*============================= Joint PD ==============================*/
/**
 * FSM State that allows PD control of the joints.
 */

#include "FSM_State_BalanceStand.h"
#include <Configuration.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND, "BALANCE_STAND"),
      _ini_jpos(12)
{
    // Do nothing here yet
}

template <typename T>
void FSM_State_BalanceStand<T>::onEnter()
{
    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();

    // Reset counter
    iter = 0;

    for (int i = 0; i < 12; i++)
    {
        _ini_jpos[i] = FSM_State<T>::_data->_legController->leg_control_data_biped.q[i];
    }

    // Reset the progress
    _progress = 0;

    printf("[FSM BALANCESTAND] On Enter\n");
    std::cout << "[FSM BALANCESTAND] joint des pos :" << this->_data->controlParameters->squat_stand_jpos[0] << " " << this->_data->controlParameters->squat_stand_jpos[1] << " " << this->_data->controlParameters->squat_stand_jpos[2] << std::endl;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_BalanceStand<T>::run()
{
    // This is just a test, should be running whatever other code you want
    Vec12<T> qDes;
    // qDes << 0.0, 0.0, 0.24, -0.48, 0.24, -0.24, 0.0, 0.0, 0.24, -0.48, -0.24, 0.24;
    qDes << 0.0, 0.0, this->_data->controlParameters->squat_stand_jpos[0], this->_data->controlParameters->squat_stand_jpos[1], this->_data->controlParameters->squat_stand_jpos[2], this->_data->controlParameters->squat_stand_jpos[2],
        0.0, 0.0, this->_data->controlParameters->squat_stand_jpos[0], this->_data->controlParameters->squat_stand_jpos[1], this->_data->controlParameters->squat_stand_jpos[2], this->_data->controlParameters->squat_stand_jpos[2];

    Vec12<T> kp_stance;
    // kp_stance << 120, 120, 120, 120, 80, 80, 120, 120, 120, 120, 80, 80;
    kp_stance << this->_data->controlParameters->stand_joint_kp[0], this->_data->controlParameters->stand_joint_kp[0], this->_data->controlParameters->stand_joint_kp[0],
        this->_data->controlParameters->stand_joint_kp[1], this->_data->controlParameters->stand_joint_kp[2], this->_data->controlParameters->stand_joint_kp[2],
        this->_data->controlParameters->stand_joint_kp[0], this->_data->controlParameters->stand_joint_kp[0], this->_data->controlParameters->stand_joint_kp[0],
        this->_data->controlParameters->stand_joint_kp[1], this->_data->controlParameters->stand_joint_kp[2], this->_data->controlParameters->stand_joint_kp[2];

    Vec12<T> kd_stance;
    // kd_stance << 3.0, 3.0, 3.0, 3.0, 0.5, 0.5, 3.0, 3.0, 3.0, 3.0, 0.5, 0.5;
    kd_stance << this->_data->controlParameters->stand_joint_kd[0], this->_data->controlParameters->stand_joint_kd[0], this->_data->controlParameters->stand_joint_kd[0],
        this->_data->controlParameters->stand_joint_kd[1], this->_data->controlParameters->stand_joint_kd[2], this->_data->controlParameters->stand_joint_kd[2],
        this->_data->controlParameters->stand_joint_kd[0], this->_data->controlParameters->stand_joint_kd[0], this->_data->controlParameters->stand_joint_kd[0],
        this->_data->controlParameters->stand_joint_kd[1], this->_data->controlParameters->stand_joint_kd[2], this->_data->controlParameters->stand_joint_kd[2];

    _progress += this->_data->controlParameters->controller_dt;
    double movement_duration(3.0);
    double ratio = _progress / movement_duration;

    if (ratio > 1.)
        ratio = 1.;

    for (int i = 0; i < 12; i++)
    {
        FSM_State<T>::_data->_legController->leg_control_data_biped.q_des[i] = ratio * qDes[i] + (1. - ratio) * _ini_jpos[i];
        FSM_State<T>::_data->_legController->leg_control_data_biped.qd_des[i] = 0.0;
        FSM_State<T>::_data->_legController->leg_control_data_biped.tau_ff[i] = 0.0;
        FSM_State<T>::_data->_legController->leg_control_data_biped.kp[i] = kp_stance[i];
        FSM_State<T>::_data->_legController->leg_control_data_biped.kd[i] = kd_stance[i];
    }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_BalanceStand<T>::checkTransition()
{
    this->nextStateName = this->stateName;

    iter++;

    // Switch FSM control mode
    switch ((int)this->_data->controlParameters->control_mode)
    {
    case K_BALANCE_STAND:
        // Normal operation for state based transitions
        break;

    case K_JOINT_PD:
        this->nextStateName = FSM_StateName::JOINT_PD;
        break;

    case K_PASSIVE:
        this->nextStateName = FSM_StateName::PASSIVE;
        this->transitionDuration = 0.0;
        break;

    case K_LOCK_JOINT:
        this->nextStateName = FSM_StateName::LOCK_JOINT;
        this->transitionDuration = 0.0;
        break;

    default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << K_BALANCE_STAND << " to "
                  << this->_data->controlParameters->control_mode << std::endl;
    }

    // Get the next state
    return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_BalanceStand<T>::transition()
{
    // Switch FSM control mode
    switch (this->nextStateName)
    {
    case FSM_StateName::JOINT_PD:
        this->transitionData.done = true;

        break;

    case FSM_StateName::PASSIVE:
        this->turnOffAllSafetyChecks();
        this->transitionData.done = true;

        break;

    case FSM_StateName::LOCK_JOINT:
        this->transitionData.done = true;

        break;

    default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << K_BALANCE_STAND << " to "
                  << this->_data->controlParameters->control_mode << std::endl;
    }
    // Finish transition
    this->transitionData.done = true;

    // Return the transition data to the FSM
    return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_BalanceStand<T>::onExit()
{
    // Nothing to clean up when exiting
}

// template class FSM_State_BalanceStand<double>;
template class FSM_State_BalanceStand<float>;
