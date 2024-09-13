/*! @file SimulationBridge.cpp
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#include "SimulationBridge.h"
#include "Utilities/SegfaultHandler.h"
#include "Controllers/LegController.h"
#include "rt/rt_rc_interface.h"
#include "rt/rt_sbus.h"

/*!
 * Connect to a simulation
 */
void SimulationBridge::run()
{
    runRobotControl();
}

/*!
 * This function handles a a control parameter message from the simulator
 */
void SimulationBridge::handleControlParameters()
{
    ControlParameterRequest &request =
        _sharedMemory().simToRobot.controlParameterRequest;
    ControlParameterResponse &response =
        _sharedMemory().robotToSim.controlParameterResponse;
    if (request.requestNumber <= response.requestNumber)
    {
        // nothing to do!
        printf(
            "[SimulationBridge] Warning: the simulator has run a ControlParameter "
            "iteration, but there is no new request!\n");
        return;
    }

    // sanity check
    u64 nRequests = request.requestNumber - response.requestNumber;
    printf("just test %ld\n", nRequests);
    assert(nRequests == 1);

    response.nParameters = _robotParams.collection._map
                               .size(); // todo don't do this every single time?

    switch (request.requestKind)
    {
    case ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME:
    {
        std::string name(request.name);
        ControlParameter &param = _robotParams.collection.lookup(name);

        // type check
        if (param._kind != request.parameterKind)
        {
            throw std::runtime_error(
                "type mismatch for parameter " + name + ", robot thinks it is " +
                controlParameterValueKindToString(param._kind) +
                " but received a command to set it to " +
                controlParameterValueKindToString(request.parameterKind));
        }

        // do the actual set
        param.set(request.value, request.parameterKind);

        // respond:
        response.requestNumber =
            request.requestNumber; // acknowledge that the set has happened
        response.parameterKind =
            request.parameterKind;      // just for debugging print statements
        response.value = request.value; // just for debugging print statements
        strcpy(response.name,
               name.c_str()); // just for debugging print statements
        response.requestKind = request.requestKind;

        printf("%s\n", response.toString().c_str());
    }
    break;

    case ControlParameterRequestKind::SET_USER_PARAM_BY_NAME:
    {
        std::string name(request.name);
        if (!_userParams)
        {
            printf("[Simulation Bridge] Warning: tried to set user parameter, but the robot does not have any!\n");
        }
        else
        {
            ControlParameter &param = _userParams->collection.lookup(name);

            // type check
            if (param._kind != request.parameterKind)
            {
                throw std::runtime_error(
                    "type mismatch for parameter " + name + ", robot thinks it is " +
                    controlParameterValueKindToString(param._kind) +
                    " but received a command to set it to " +
                    controlParameterValueKindToString(request.parameterKind));
            }

            // do the actual set
            param.set(request.value, request.parameterKind);
        }

        // respond:
        response.requestNumber =
            request.requestNumber; // acknowledge that the set has happened
        response.parameterKind =
            request.parameterKind;      // just for debugging print statements
        response.value = request.value; // just for debugging print statements
        strcpy(response.name,
               name.c_str()); // just for debugging print statements
        response.requestKind = request.requestKind;

        printf("%s\n", response.toString().c_str());
    }
    break;

    case ControlParameterRequestKind::GET_ROBOT_PARAM_BY_NAME:
    {
        std::string name(request.name);
        ControlParameter &param = _robotParams.collection.lookup(name);

        // type check
        if (param._kind != request.parameterKind)
        {
            throw std::runtime_error(
                "type mismatch for parameter " + name + ", robot thinks it is " +
                controlParameterValueKindToString(param._kind) +
                " but received a command to set it to " +
                controlParameterValueKindToString(request.parameterKind));
        }

        // respond
        response.value = param.get(request.parameterKind);
        response.requestNumber = request.requestNumber; // acknowledge
        response.parameterKind =
            request.parameterKind; // just for debugging print statements
        strcpy(response.name,
               name.c_str()); // just for debugging print statements
        response.requestKind =
            request.requestKind; // just for debugging print statements

        printf("%s\n", response.toString().c_str());
    }
    break;
    default:
        throw std::runtime_error("unhandled get/set");
    }
}

/*!
 * Run the robot controller
 */
void SimulationBridge::runRobotControl()
{
    if (_firstControllerRun)
    {
        printf("Begin init robot runner\n");

        _robotRunner->robotType = _robot;
        _robotParams.controller_dt = TIME_STEP / 1000.0; // 0.002;
        _robotParams.control_mode = 0;
        _robotParams.use_rc = 1;
        _robotRunner->controlParameters = &_robotParams;
        _robotRunner->visualizationData = &local_visualization;

        _robotRunner->init();
        _firstControllerRun = false;

        sbus_thread = new std::thread(&SimulationBridge::run_sbus, this);
    }
    _robotRunner->run();
}

/*!
 * Run the RC receive thread
 */
void SimulationBridge::run_sbus()
{
    // billchen edit once
    printf("[run_sbus] starting...\n");
    int port = init_sbus(true); // Simulation

    while (true)
    {
        // printf("[run_sbus] run \n");
        if (port > 0)
        {
            int x = receive_sbus(port);
            //      printf("[sbus] receive sbus %d\n",x);
            if (x)
            {
                sbus_packet_complete_at9s();
            }
        }
        usleep(5000);
    }
}
