//
// Created by billchent on 2020/9/23.
//
/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_RL_Locomotion.h"
#include <Utilities/Timer.h>
// #include <rt/rt_interface_lcm.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_RL_Locomotion<T>::FSM_State_RL_Locomotion(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::RL_LOCOMOTION, "RL_LOCOMOTION")
{
    rlActualController = new LearningBasedController(_controlFSMData);

    this->turnOnAllSafetyChecks();
    // Turn off Foot pos command since it is set in WBC as operational task
    this->checkPDesFoot = false;
}

template <typename T>
void FSM_State_RL_Locomotion<T>::onEnter()
{
    // Default is to not transition
    this->nextStateName = this->stateName;
    // Reset the transition data
    this->transitionData.zero();
    // initializ the RL algorithm
    rlActualController->init();

    printf("[FSM RL_LOCOMOTION] On Enter\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_RL_Locomotion<T>::run()
{
    Timer t1;
    LocomotionControlStep();
    //    printf("MPC_WBC Solve time %f ms\n", t1.getMs());
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_RL_Locomotion<T>::checkTransition()
{
    // Get the next state
    iter++;

    // Switch FSM control mode
    if (locomotionSafe())
    {
        switch ((int)this->_data->controlParameters->control_mode)
        {
        case K_RL_LOCOMOTION:
            break;

        case K_PASSIVE:
            this->nextStateName = FSM_StateName::PASSIVE;
            // Transition time is immediate
            this->transitionDuration = 0.0;
            break;

        case K_LOCK_JOINT:
            this->nextStateName = FSM_StateName::LOCK_JOINT;
            this->transitionDuration = 0.;
            break;

        case K_JOINT_PD:
            this->nextStateName = FSM_StateName::JOINT_PD;
            this->transitionDuration = 0;
            break;

        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_RL_LOCOMOTION << " to "
                      << this->_data->controlParameters->control_mode << std::endl;
        }
    }
    else
    {
        this->nextStateName = FSM_StateName::LOCK_JOINT;
        this->transitionDuration = 0.;
        printf("BILLCHEN RINTF: rl locomotion force to LockJoint\n");
    }

    // Return the next state name to the FSM
    return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_RL_Locomotion<T>::transition()
{
    // Switch FSM control mode
    switch (this->nextStateName)
    {
    case FSM_StateName::PASSIVE:
        this->turnOffAllSafetyChecks();
        this->transitionData.done = true;
        break;

    case FSM_StateName::LOCK_JOINT:
        this->transitionData.done = true;
        break;

    case FSM_StateName::JOINT_PD:
        this->transitionData.done = true;
        break;

    default:
        std::cout << "[CONTROL FSM] Something went wrong in transition"
                  << std::endl;
    }

    // Return the transition data to the FSM
    return this->transitionData;
}

template <typename T>
bool FSM_State_RL_Locomotion<T>::locomotionSafe()
{

    return true;

    T max_leg_y_offset = 0.18;
    T max_roll = 40;
    T max_pitch = 40;
    T max_v_leg = 9;
    auto &seResult = this->_data->_stateEstimator->getResult();
    if (this->_data->_humanoid_biped->_robotType == RobotType::ZQ_Biped_SA01)
    {
        max_roll = 80;
        max_pitch = 80;
        max_leg_y_offset = 0.28 * 2;
        max_v_leg = 19;
    }
    else // if(this->_data->_humanoid_biped->_robotType ==  RobotType::ZQ_Biped_SA01P)
    {
        max_roll = 50;
        max_pitch = 60;
        max_leg_y_offset = 0.3;
        max_v_leg = 19;
    }

    if (std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll))
    {
        printf("Unsafe rl locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
        return false;
    }

    if (std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch))
    {
        printf("Unsafe rl locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
        return false;
    }

    for (int leg = 0; leg < 2; leg++)
    {
        auto p_leg = this->_data->_legController->datas[leg].p;
        if (p_leg[2] > 0)
        {
            printf("Unsafe rl locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
            return false;
        }

        if (std::fabs(p_leg[1] > max_leg_y_offset))
        {
            printf("Unsafe rl locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
            return false;
        }

        auto v_leg = this->_data->_legController->datas[leg].v.norm();
        if (std::fabs(v_leg) > max_v_leg)
        {
            printf("Unsafe rl locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
            return false;
        }
    }

    return true;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_RL_Locomotion<T>::onExit()
{
    // Nothing to clean up when exiting
    iter = 0;
}

/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_RL_Locomotion<T>::LocomotionControlStep()
{
    rlActualController->update();
}

// template class FSM_State_RL_Locomotion<double>;
template class FSM_State_RL_Locomotion<float>;
