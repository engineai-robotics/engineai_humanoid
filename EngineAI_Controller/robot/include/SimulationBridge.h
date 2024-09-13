//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_SIMULATIONBRIDGE_H
#define ZQ_HUMANOID_SIMULATIONBRIDGE_H

#include <thread>

#include "../../common/include/ControlParameters/RobotParameters.h"
#include "../include/RobotRunner.h"
#include "../../common/include/SimUtilities/SimulatorMessage.h"
#include "../../common/include/Types.h"
#include "../../common/include/Utilities/PeriodicTask.h"
#include "../../common/include/Utilities/SharedMemory.h"

class SimulationBridge
{
public: // explicit
    SimulationBridge(RobotType robot, RobotController *robot_ctrl) : _robot(robot)
    {
        _fakeTaskManager = new PeriodicTaskManager;
        _robotRunner = new RobotRunner(robot_ctrl, _fakeTaskManager, 0, "robot-task");
        _userParams = robot_ctrl->getUserControlParameters();
    }
    void run();
    void handleControlParameters();
    void runRobotControl();
    ~SimulationBridge()
    {
        delete _fakeTaskManager;
        delete _robotRunner;
    }
    void run_sbus();
    void at9s_sbus();

private:
    PeriodicTaskManager taskManager;
    bool _firstControllerRun = true;
    PeriodicTaskManager *_fakeTaskManager = nullptr;
    RobotType _robot;
    RobotRunner *_robotRunner = nullptr;
    SimulatorMode _simMode;
    SharedMemoryObject<SimulatorSyncronizedMessage> _sharedMemory;
    RobotControlParameters _robotParams;
    VisualizationData local_visualization;
    ControlParameters *_userParams = nullptr;
    u64 _iterations = 0;

    std::thread *sbus_thread;
    int at9s_port;
};

#endif // ZQ_HUMANOID_SIMULATIONBRIDGE_H
