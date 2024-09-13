//
// Created by engineai on 2024/07/03.
//
/*============================== Passive ==============================*/
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

#include "FSM_State_Estop.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Estop<T>::FSM_State_Estop(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::ESTOP, "ESTOP")
{
    // Do nothing
    // Set the pre controls safety checks
    this->checkSafeOrientation = false;

    // Post control safety checks
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_Estop<T>::onEnter()
{
    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();

    printf("[FSM ESTOP] On Enter\n");
    // set_motor_enable_cmd(0, 12);

    *(this->_data->_legController->motor_disable_flag) = true;

    if (!set_motor_enable_cmd(/*enabled*/ false, /*total_joints*/ 12))
        *(this->_data->_legController->motor_disable_flag) = false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Estop<T>::run()
{
    // Do nothing, all commands should begin as zeros
    if (*(this->_data->_legController->motor_disable_flag))
        if (!set_motor_enable_cmd(/*enabled*/ false, /*total_joints*/ 12))
            *(this->_data->_legController->motor_disable_flag) = false;

    static int count = 0;
    if (count % 1000 == 0)
    {
        std::cout << "[FSM ESTOP] Motors are disabled" << std::endl;
        count = 0;
    }
    count++;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Estop<T>::testTransition()
{
    this->transitionData.done = true;
    return this->transitionData;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Estop<T>::checkTransition()
{
    this->nextStateName = this->stateName;
    iter++;

    // Switch FSM control mode
    switch ((int)this->_data->controlParameters->control_mode)
    {
    case K_ESTOP: // normal c (0)
        // Normal operation for state based transitions
        break;

    case K_PASSIVE: // normal c (0)
        // Normal operation for state based transitions
        this->nextStateName = FSM_StateName::PASSIVE;
        break;

    default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << K_ESTOP << " to "
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
TransitionData<T> FSM_State_Estop<T>::transition()
{
    // Finish Transition
    this->transitionData.done = true;

    // Return the transition data to the FSM
    return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Estop<T>::onExit()
{
    // Nothing to clean up when exiting
}

// template class FSM_State_Estop<double>;
template class FSM_State_Estop<float>;
