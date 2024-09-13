/*!
 * @file HardwareBridge.cpp
 * @brief Interface between robot control algorithm and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */
#ifdef linux

#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include "Configuration.h"

#include "HardwareBridge.h"
#include "rt/rt_sbus.h"
#include "Utilities/Utilities_print.h"

#define PI 3.1415
#define JOINT_TEST

// motor zero calibration
// first left and then right
const float joint_direction[12] = {1.f, -1.f, -1.f, -1.f, -1.f, 1.f,
                                   1.f, -1.f, 1.f, 1.f, 1.f, -1.f};

// SA01 - 1 motor zero offset
const float joint_zero_offset[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// SA01 - 2 motor zero offset
// const float joint_zero_offset_right_leg[6] = {-0.387007, 2.06664, -3.10235, -0.0368128, -1.93694, -0.0497828};
// const float joint_zero_offset_left_leg[6] = {2.49199, 0.0207901, 3.12257, 0.015831, 1.90681, 0.223354};

/*!
 * If an error occurs during initialization, before motors are enabled, print
 * error and exit.
 * @param reason Error message string
 * @param printErrno If true, also print C errno
 */
void HardwareBridge::initError(const char *reason, bool printErrno)
{
    printf("FAILED TO INITIALIZE HARDWARE: %s\n", reason);

    if (printErrno)
    {
        printf("Error: %s\n", strerror(errno));
    }

    exit(-1);
}

/*!
 * All hardware initialization steps that are common for all robot series
 */
void HardwareBridge::initCommon()
{
    printf("[HardwareBridge] Init stack\n");
    //    prefaultStack();
    printf("[HardwareBridge] Init scheduler\n");
    setupScheduler();
    if (!_interfaceLCM.good())
    {
        initError("_interfaceLCM failed to initialize\n", false);
    }
    else
        std::cout << "_interfaceLCM OK" << std::endl;

    printf("[HardwareBridge] Subscribe LCM\n");
    _interfaceLCM.subscribe("interface", &HardwareBridge::handleGamepadLCM, this);
    _interfaceLCM.subscribe("interface_request", &HardwareBridge::handleControlParameter, this);

    printf("[HardwareBridge] Start interface LCM handler\n");

    _interfaceLcmThread = std::thread(&HardwareBridge::handleInterfaceLCM, this);
}

/*!
 * Run interface LCM
 */
void HardwareBridge::handleInterfaceLCM()
{
    while (!_interfaceLcmQuit)
    {
        _interfaceLCM.handle();
    }
}

/*!
 * Writes to a 16 KB buffer on the stack. If we are using 4K pages for our
 * stack, this will make sure that we won't have a page fault when the stack
 * grows.  Also mlock's all pages associated with the current process, which
 * prevents the engineai_biped software from being swapped out.  If we do run out of
 * memory, the robot program will be killed by the OOM process killer (and
 * leaves a log) instead of just becoming unresponsive.
 */
// 分配stack 防止空间被swap，提高运行效率
void HardwareBridge::prefaultStack()
{
    printf("[Init] Prefault stack...\n");
    volatile char stack[MAX_STACK_SIZE];
    memset(const_cast<char *>(stack), 0, MAX_STACK_SIZE);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        initError("mlockall failed.  This is likely because you didn't run robot as root.\n", true);
    }
}

/*!
 * Configures the scheduler for real time priority
 */
// 进程优先级
void HardwareBridge::setupScheduler()
{
    printf("[Init] Setup RT Scheduler...\n");
    struct sched_param params;
    params.sched_priority = TASK_PRIORITY; // 设置进程调度的优先级
    if (sched_setscheduler(0, SCHED_FIFO, &params) == -1)
    { // pid为零，将会为调用进程设置调度策略和调度参数
        initError("sched_setscheduler failed.\n", true);
    }
}

/*!
 * LCM Handler for gamepad message
 */
void HardwareBridge::handleGamepadLCM(const lcm::ReceiveBuffer *rbuf,
                                      const std::string &chan,
                                      const gamepad_lcmt *msg)
{
    (void)rbuf;
    (void)chan;
    _gamepadCommand.set(msg);
}

/*!
 * LCM Handler for control parameters
 */
void HardwareBridge::handleControlParameter(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const control_parameter_request_lcmt *msg)
{
    (void)rbuf;
    (void)chan;
    if (msg->requestNumber <= _parameter_response_lcmt.requestNumber)
    {
        // nothing to do!
        printf(
            "[HardwareBridge] Warning: the interface has run a ControlParameter "
            "iteration, but there is no new request!\n");
        // return;
    }

    // sanity check
    s64 nRequests = msg->requestNumber - _parameter_response_lcmt.requestNumber;
    if (nRequests != 1)
    {
        printf("[ERROR] Hardware bridge: we've missed %ld requests\n",
               nRequests - 1);
    }

    switch (msg->requestKind)
    {
    case (s8)ControlParameterRequestKind::SET_USER_PARAM_BY_NAME:
    {
        if (!_userControlParameters)
        {
            printf("[Warning] Got user param %s, but not using user parameters!\n",
                   (char *)msg->name);
        }
        else
        {
            std::string name((char *)msg->name);
            ControlParameter &param = _userControlParameters->collection.lookup(name);

            // type check
            if ((s8)param._kind != msg->parameterKind)
            {
                throw std::runtime_error(
                    "type mismatch for parameter " + name + ", robot thinks it is " +
                    controlParameterValueKindToString(param._kind) +
                    " but received a command to set it to " +
                    controlParameterValueKindToString(
                        (ControlParameterValueKind)msg->parameterKind));
            }

            // do the actual set
            ControlParameterValue v;
            memcpy(&v, msg->value, sizeof(v));
            param.set(v, (ControlParameterValueKind)msg->parameterKind);

            // respond:
            _parameter_response_lcmt.requestNumber =
                msg->requestNumber; // acknowledge that the set has happened
            _parameter_response_lcmt.parameterKind =
                msg->parameterKind; // just for debugging print statements
            memcpy(_parameter_response_lcmt.value, msg->value, 64);
            //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
            // for debugging print statements
            strcpy((char *)_parameter_response_lcmt.name,
                   name.c_str()); // just for debugging print statements
            _parameter_response_lcmt.requestKind = msg->requestKind;

            printf("[User Control Parameter] set %s to %s\n", name.c_str(),
                   controlParameterValueToString(
                       v, (ControlParameterValueKind)msg->parameterKind)
                       .c_str());
        }
    }
    break;

    case (s8)ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME:
    {
        std::string name((char *)msg->name);
        ControlParameter &param = _robotParams.collection.lookup(name);

        // type check
        if ((s8)param._kind != msg->parameterKind)
        {
            throw std::runtime_error(
                "type mismatch for parameter " + name + ", robot thinks it is " +
                controlParameterValueKindToString(param._kind) +
                " but received a command to set it to " +
                controlParameterValueKindToString(
                    (ControlParameterValueKind)msg->parameterKind));
        }

        // do the actual set
        ControlParameterValue v;
        memcpy(&v, msg->value, sizeof(v));
        param.set(v, (ControlParameterValueKind)msg->parameterKind);

        // respond:
        _parameter_response_lcmt.requestNumber =
            msg->requestNumber; // acknowledge that the set has happened
        _parameter_response_lcmt.parameterKind =
            msg->parameterKind; // just for debugging print statements
        memcpy(_parameter_response_lcmt.value, msg->value, 64);
        //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
        // for debugging print statements
        strcpy((char *)_parameter_response_lcmt.name,
               name.c_str()); // just for debugging print statements
        _parameter_response_lcmt.requestKind = msg->requestKind;

        printf("[Robot Control Parameter] set %s to %s\n", name.c_str(),
               controlParameterValueToString(
                   v, (ControlParameterValueKind)msg->parameterKind)
                   .c_str());
    }
    break;

    default:
    {
        throw std::runtime_error("parameter type unsupported");
    }
    break;
    }
    _interfaceLCM.publish("interface_response", &_parameter_response_lcmt);
}

ZqSA01HardwareBridge::ZqSA01HardwareBridge(RobotController *robot_ctrl, bool load_parameters_from_file)
    : HardwareBridge(robot_ctrl),
      _spiLcm(getLcmUrl(255))
{
    _load_parameters_from_file = load_parameters_from_file;
}

/*!
 * Main method for engineai robot hardware
 */
void ZqSA01HardwareBridge::run()
{
    initCommon();
    initHardware();
    loadConfigParam();

    printf("[Hardware Bridge] Finished all initialization, start to run!\n");

    _firstRun = false;

    /* thread registration*/
    //  motor comm registration
    PeriodicMemberFunction<ZqSA01HardwareBridge> spiTask(
        &taskManager, .002, "spi", &ZqSA01HardwareBridge::runSpi, this);

    // yesense IMU task
    if (yesense_imu_init)
        _yesenseImuThread = std::thread(&ZqSA01HardwareBridge::runYesenseIMU, this);

    // dynamic loading yaml files
    // PeriodicMemberFunction<ZqSA01HardwareBridge> ParamLoadingTask(
    //     &taskManager, 5.0, "paramDynamicLoading", &ZqSA01HardwareBridge::runParamFileLoading, this);

    // robot controller
    _robotRunner =
        new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

    _robotRunner->driverCommand = &_gamepadCommand;
    _robotRunner->jointDatas = &_joint_state;
    _robotRunner->jointCommand = &_joint_cmd;
    _robotRunner->robotType = RobotType::ZQ_Biped_SA01;
    _robotRunner->vectorNavData = &_vectorNavData;
    _robotRunner->controlParameters = &_robotParams;
    _robotRunner->visualizationData = &_visualizationData;
    _robotRunner->engineAIRobotVisualization = _engineAIRobotVisualization;

    // gamepad task
    PeriodicMemberFunction<HardwareBridge> sbusTask(
        &taskManager, .100, "rc_controller", &HardwareBridge::run_sbus, this);

    /*thread start*/
    statusTask.start(); // status watching

    spiTask.start(); // motor comm thread

    // ParamLoadingTask.start(); //  yaml dynamic loading thread

    sbusTask.start(); // gamepad thread

    _robotRunner->start(); // controller thread

    // main thread loop
    for (;;)
    {
        usleep(10000000); // 10s
    }
}

/*!
 * Receive RC with SBUS
 */
void HardwareBridge::run_sbus()
{
    if (_port > 0)
    {
        int x = receive_sbus(_port);

        if (x)
        {
#ifdef K_USE_LOGITECH_GAMEPAD
            sbus_packet_complete_logitech();
#elif RC_AT9s
            sbus_packet_complete_at9s();
#else
            sbus_packet_complete();
#endif
        }
    }
}

void ZqSA01HardwareBridge::runYesenseIMU()
{
    while (true)
    {
        yesense_imu->run();

        _vectorNavData.accelerometer = yesense_imu->acc;

        _vectorNavData.gyro = yesense_imu->gyro;

        _vectorNavData.quat[0] = yesense_imu->quat[1]; // x
        _vectorNavData.quat[1] = yesense_imu->quat[2]; // y
        _vectorNavData.quat[2] = yesense_imu->quat[3]; // z
        _vectorNavData.quat[3] = yesense_imu->quat[0]; // w

        // std::cout << "IMU: " << yesense_imu->rpy.transpose() << std::endl;
    }
}

void ZqSA01HardwareBridge::runParamFileLoading()
{
    try
    {
        _robotParams.initializeFromYamlFile("../config/zqsa01-robot-default-param.yaml");
    }
    catch (std::exception &e)
    {
        printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
        exit(1);
    }
    std::cout << "dynamic loading yaml file succeed" << std::endl;
}

/*!
 * Initialize specific hardware for ZQ_SA01
 */
void ZqSA01HardwareBridge::initHardware()
{
    _vectorNavData.quat << 1, 0, 0, 0;

    int ret = -1;

    ret = motor_init();

    if (ret)
    {
        std::cout << "motor init failed" << std::endl;
        exit(1);
    }

    // init for IMU
    yesense_imu = new yesense::YesenseDriver(); // initialization on construction
    yesense_imu_init = yesense_imu->getConfiguration();

    std::cout << "yesense_imu_init: " << yesense_imu_init << std::endl;

    if (yesense_imu_init)
        printf("[HardwareBridge] yesenseIMUInit Success\n");
    else
        printf("[HardwareBridge] yesenseIMUInit Failed\n");

    // init gamepad
    _port = init_sbus(false); // false for real robot, and true for Simulation
}

void ZqSA01HardwareBridge ::loadConfigParam()
{
    try
    {
        _robotParams.initializeFromYamlFile("../config/zqsa01-robot-default-param.yaml");
    }
    catch (std::exception &e)
    {
        printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
        exit(1);
    }
    _robotParams.controller_dt = TIME_STEP / 1000.0; // 0.002;
    _robotParams.control_mode = 0;

    _robotParams.use_rc = 1;

    std::cout << "load yaml file succed" << std::endl;
}

/*!
 * Run SPI
 */
void ZqSA01HardwareBridge::runSpi()
{
    if (!_robotRunner->motor_enable_flag && !_robotRunner->motor_disable_flag)
    {
        // original motor states and cmd
        motor_data = get_motor_data();
        motor_cmd = get_motor_cmd();

        // joint states and cmd (direction and zero calibration)
        for (int i = 0; i < 12; i++)
        {
            // joint states
            _joint_state.q[i] = (motor_data->q[i] - joint_zero_offset[i]) * joint_direction[i];
            _joint_state.qd[i] = motor_data->qd[i] * joint_direction[i];
            _joint_state.tau[i] = motor_data->tau[i] * joint_direction[i];

            // joint cmd
            motor_cmd->q_des[i] = _joint_cmd.q_des[i] * joint_direction[i] + joint_zero_offset[i];
            motor_cmd->qd_des[i] = _joint_cmd.qd_des[i] * joint_direction[i];
            motor_cmd->tau_ff[i] = _joint_cmd.tau_ff[i] * joint_direction[i];
            motor_cmd->kp[i] = _joint_cmd.kp[i];
            motor_cmd->kd[i] = _joint_cmd.kd[i];

            // lcm for joint datas
            joint_data.q[i] = _joint_state.q[i];
            joint_data.qd[i] = _joint_state.qd[i];
            joint_data.tau_est[i] = _joint_state.tau[i];

            joint_cmd.q_des[i] = _joint_cmd.q_des[i];
            joint_cmd.qd_des[i] = _joint_cmd.qd_des[i];
            joint_cmd.kp_joint[i] = _joint_cmd.kp[i];
            joint_cmd.kd_joint[i] = _joint_cmd.kd[i];
            joint_cmd.tau_ff[i] = _joint_cmd.tau_ff[i];
        }

        set_motor_cmd(motor_cmd);
    }

    _spiLcm.publish("joint_data", &joint_data);
    _spiLcm.publish("joint_cmd", &joint_cmd);
}

#endif
