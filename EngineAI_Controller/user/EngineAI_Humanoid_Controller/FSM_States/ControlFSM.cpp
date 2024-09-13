//
// Created by engineai on 2024/07/03.
//
/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM.h"
#include "../../robot/include/rt/rt_rc_interface.h"
// #include <rt/rt_rc_interface.h>

/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param _humanoid_biped the humanoid_biped information
 * @param _stateEstimator contains the estimated states
 * @param _legController interface to the leg controllers
 * @param _gaitScheduler controls scheduled foot contact modes
 * @param _desiredStateCommand gets the desired COM state trajectories
 * @param controlParameters passes in the control parameters from the GUI
 */
template <typename T>
ControlFSM<T>::ControlFSM(RobotConstructor<T> *_humanoid_biped,
                          StateEstimatorContainer<T> *_stateEstimator,
                          LegController<T> *_legController,
                          GaitScheduler<T> *_gaitScheduler,
                          DesiredStateCommand<T> *_desiredStateCommand,
                          RobotControlParameters *controlParameters,
                          VisualizationData *visualizationData,
                          EngineAI_UserParameters *userParameters)
{
    // Add the pointers to the ControlFSMData struct
    data._humanoid_biped = _humanoid_biped;
    data._stateEstimator = _stateEstimator;
    data._legController = _legController;
    data._gaitScheduler = _gaitScheduler;
    data._desiredStateCommand = _desiredStateCommand;
    data.controlParameters = controlParameters;
    data.visualizationData = visualizationData;
    data.userParameters = userParameters;

    // Initialize and add all of the FSM States to the state list
    statesList.estop = new FSM_State_Estop<T>(&data);
    statesList.passive = new FSM_State_Passive<T>(&data);
    statesList.jointPD = new FSM_State_JointPD<T>(&data);
    statesList.balanceStand = new FSM_State_BalanceStand<T>(&data);

    statesList.lockjoint = new FSM_State_LockJoint<T>(&data);
    statesList.RL_locomotion = new FSM_State_RL_Locomotion<T>(&data);
    statesList.invalid = nullptr;

    safetyChecker = new SafetyChecker<T>(&data);

    // Initialize the FSM with the Passive FSM State
    initialize();
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */
template <typename T>
void ControlFSM<T>::initialize()
{
    // Initialize a new FSM State with the control data
    currentState = statesList.estop;
    //    currentState = statesList.VM_WBC_locomotion;

    // Enter the new current state cleanly
    currentState->onEnter();

    // Initialize to not be in transition
    nextState = currentState;

    // Initialize FSM mode to normal operation
    operatingMode = FSM_OperatingMode::NORMAL;
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template <typename T>
void ControlFSM<T>::runFSM()
{
    // Check the robot state for safe operation
    operatingMode = safetyPreCheck();

    if (data.controlParameters->use_rc)
    {
        int rc_mode = data._desiredStateCommand->rcCommand->mode;

        if (rc_mode == RC_mode::OFF)
        {
            data.controlParameters->control_mode = K_ESTOP; // no control
        }
        else if (rc_mode == RC_mode::PASSIVE)
        {
            data.controlParameters->control_mode = K_PASSIVE; // no control
        }
        else if (rc_mode == RC_mode::STAND_UP)
        {
            data.controlParameters->control_mode = K_JOINT_PD; // straight stand
        }
        else if (rc_mode == RC_mode::LOCK_JOINT)
        {
            data.controlParameters->control_mode = K_LOCK_JOINT; // safety damping
        }
        else if (rc_mode == RC_mode::LOCOMOTION)
        {
            data.controlParameters->control_mode = K_RL_LOCOMOTION; // locomotion by RL method
        }
        else if (rc_mode == RC_mode::BALANCE_STAND)
        {
            data.controlParameters->control_mode = K_BALANCE_STAND; // squat stand
        }
        else // keep the current mode
        {
            assert(false);
        }
    }

    static int count = 0;
    count++;
    if (count % 2000 == 0)
    {
        std::cout << "\033[1;33m operatingMode=\n"
                  << static_cast<int>(operatingMode) << " \033[0m \n";
        count = 0;
    }

    // Run the robot control code if operating mode is not unsafe
    if (operatingMode != FSM_OperatingMode::ESTOP)
    {
        // Run normal controls if no transition is detected
        if (operatingMode == FSM_OperatingMode::NORMAL)
        {
            // Check the current state for any transition
            nextStateName = currentState->checkTransition();

            static int count_state_name = 0;
            count_state_name++;
            if (count_state_name % 2000 == 0)
            {
                std::cout << "\033[1;33m nextStateName=\n"
                          << static_cast<int>(nextStateName) << " \033[0m \n";
                count_state_name = 0;
            }

            // Detect a commanded transition
            if (nextStateName != currentState->stateName)
            {
                // Set the FSM operating mode to transitioning
                operatingMode = FSM_OperatingMode::TRANSITIONING;

                // Get the next FSM State by name
                nextState = getNextState(nextStateName);
                // std::cout << "nextState: " << static_cast<int>(nextState) << std::endl;

                // Print transition initialized info
                printInfo(1);
            }
            else
            {
                // Run the iteration for the current state normally
                //                std::cout<<"currentState->run();"<<std::endl;
                currentState->run();
            }
        }

        // Run the transition code while transition is occuring
        if (operatingMode == FSM_OperatingMode::TRANSITIONING)
        {
            transitionData = currentState->transition();

            // Check the robot state for safe operation
            // safetyPostCheck();

            // Run the state transition
            if (transitionData.done)
            {
                // Exit the current state cleanly
                currentState->onExit();

                // Print finalizing transition info
                // printInfo(2);

                // Complete the transition
                currentState = nextState;

                // Enter the new current state cleanly
                currentState->onEnter();

                // Return the FSM to normal operation mode
                operatingMode = FSM_OperatingMode::NORMAL;
            }
        }
        else
        {
            // Check the robot state for safe operation
            safetyPostCheck();
        }
    }
    else
    { // if ESTOP
        currentState = statesList.estop;
        currentState->onEnter();
        nextStateName = currentState->stateName;
    }

    // Print the current state of the FSM
    printInfo(0);

    // Increase the iteration counter
    iter++;
}

/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 *
 * @return the appropriate operating mode
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck()
{
    // Check for safe orientation if the current state requires it
    if (currentState->checkSafeOrientation && data.controlParameters->control_mode != K_LOCK_JOINT)
    {
        if (!safetyChecker->checkSafeOrientation())
        {
            operatingMode = FSM_OperatingMode::ESTOP;
            std::cout << "broken: Orientation Safety Ceck FAIL" << std::endl;
        }
    }

    // Default is to return the current operating mode
    return operatingMode;
}

/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 *
 * @return the appropriate operating mode
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck()
{
    // Check for safe desired foot positions
    if (currentState->checkPDesFoot)
    {
        safetyChecker->checkPDesFoot();
    }

    // Check for safe desired feedforward forces
    if (currentState->checkForceFeedForward)
    {
        safetyChecker->checkForceFeedForward();
    }

    // Default is to return the current operating mode
    return operatingMode;
}

/**
 * Returns the approptiate next FSM State when commanded.
 *
 * @param  next commanded enumerated state name
 * @return next FSM state
 */
template <typename T>
FSM_State<T> *ControlFSM<T>::getNextState(FSM_StateName stateName)
{
    // Choose the correct FSM State by enumerated state name
    switch (stateName)
    {
    case FSM_StateName::INVALID:
        return statesList.invalid;

    case FSM_StateName::PASSIVE:
        return statesList.passive;

    case FSM_StateName::JOINT_PD:
        return statesList.jointPD;

    case FSM_StateName::RL_LOCOMOTION:
        return statesList.RL_locomotion;

    case FSM_StateName::BALANCE_STAND:
        return statesList.balanceStand;

    case FSM_StateName::LOCK_JOINT:
        return statesList.lockjoint;

    case FSM_StateName::ESTOP:
        return statesList.estop;

    default:
        return statesList.invalid;
    }
}

/**
 * Prints Control FSM info at regular intervals and on important events
 * such as transition initializations and finalizations. Separate function
 * to not clutter the actual code.
 *
 * @param printing mode option for regular or an event
 */
template <typename T>
void ControlFSM<T>::printInfo(int opt)
{
    switch (opt)
    {
    case 0: // Normal printing case at regular intervals
        // Increment printing iteration
        printIter++;

        // Print at commanded frequency
        if (printIter == printNum)
        {
            std::cout << "[CONTROL FSM] Printing FSM Info...\n";
            std::cout
                << "---------------------------------------------------------\n";
            std::cout << "Iteration: " << iter << std::endl;
            if (operatingMode == FSM_OperatingMode::NORMAL)
            {
                std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                          << std::endl;
            }
            else if (operatingMode == FSM_OperatingMode::TRANSITIONING)
            {
                std::cout << "Operating Mode: TRANSITIONING from "
                          << currentState->stateString << " to "
                          << nextState->stateString << std::endl;
            }
            else if (operatingMode == FSM_OperatingMode::ESTOP)
            {
                std::cout << "Operating Mode: ESTOP\n";
            }
            std::cout << "Gait Type: " << data._gaitScheduler->gaitData.gaitName
                      << std::endl;

            // Reset iteration counter
            printIter = 0;
        }

        break;

    case 1: // Initializing FSM State transition
        std::cout << "[CONTROL FSM] Transition initialized from "
                  << currentState->stateString << " to " << nextState->stateString
                  << std::endl;

        break;

    case 2: // Finalizing FSM State transition
        std::cout << "[CONTROL FSM] Transition finalizing from "
                  << currentState->stateString << " to " << nextState->stateString
                  << std::endl;

        break;
    }
}

// template class ControlFSM<double>; This should be fixed... need to make
// RobotRunner a template
template class ControlFSM<float>;
