//
// Created by engineai on 2024/07/03.
//
#include "EngineAI_Humanoid_Controller.h"

EngineAI_Humanoid_Controller::EngineAI_Humanoid_Controller() : RobotController()
{
}

/**
 * Initializes the Control FSM.
 */
void EngineAI_Humanoid_Controller::initializeController()
{
    // Initialize a new GaitScheduler object
    _gaitScheduler = new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);

    // Initializes the Control FSM with all the required data
    _controlFSM = new ControlFSM<float>(_humanoid_biped, _stateEstimator,
                                        _legController, _gaitScheduler,
                                        _desiredStateCommand, _controlParameters,
                                        _visualizationData, &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void EngineAI_Humanoid_Controller::runController()
{
    // Find the desired state trajectory
    _desiredStateCommand->convertToStateCommands();
    // Run the Control FSM code
    _controlFSM->runFSM();
}
