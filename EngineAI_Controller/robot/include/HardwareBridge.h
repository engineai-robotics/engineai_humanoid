//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_HARDWAREBRIDGE_H
#define ZQ_HUMANOID_HARDWAREBRIDGE_H

#ifdef linux

#define MAX_STACK_SIZE 16384 // 16KB  of stack
#define TASK_PRIORITY 49     // linux priority, this is not the nice value

#include <string>
#include <lcm/lcm-cpp.hpp>
#include <third-party/yesense_without_ros/src/yesense_driver.h>

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "leg_control_command_lcmt.hpp"
#include "motor.h"

/*!
 * Interface between robot and hardware
 */
class HardwareBridge
{
public:
    HardwareBridge(RobotController *robot_ctrl)
        : statusTask(&taskManager, 0.5f),
          _interfaceLCM(getLcmUrl(255))
    {
        _controller = robot_ctrl;
    }
    void prefaultStack();
    void setupScheduler();
    void initError(const char *reason, bool printErrno = false);
    void initCommon();
    ~HardwareBridge() { delete _robotRunner; }
    void handleGamepadLCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                          const gamepad_lcmt *msg);

    void handleInterfaceLCM();
    void handleControlParameter(const lcm::ReceiveBuffer *rbuf,
                                const std::string &chan,
                                const control_parameter_request_lcmt *msg);
    void run_sbus();

protected:
    PeriodicTaskManager taskManager;
    PrintTaskStatus statusTask;
    GamepadCommand _gamepadCommand;
    VisualizationData _visualizationData;
    EngineAIRobotVisualization _engineAIRobotVisualization;

    motor_data_t _motorDatas;
    motor_command_t _motorCommand;
    TiBoardCommand _tiBoardCommand[4];
    TiBoardData _tiBoardData[4];
    bool _firstRun = true;
    RobotRunner *_robotRunner = nullptr;
    RobotControlParameters _robotParams;
    u64 _iterations = 0;
    std::thread _interfaceLcmThread;
    volatile bool _interfaceLcmQuit = false;
    RobotController *_controller = nullptr;
    ControlParameters *_userControlParameters = nullptr;
    bool get_param_from_yaml = false;
    int _port;
    motor_data_t _joint_state;
    motor_command_t _joint_cmd;
    motor_data_t *motor_data = nullptr;
    motor_command_t *motor_cmd = nullptr;

    lcm::LCM _interfaceLCM;
    control_parameter_respones_lcmt _parameter_response_lcmt;

    leg_control_data_lcmt joint_data;
    leg_control_command_lcmt joint_cmd;
};

/*!
 * Interface between robot and hardware
 */
class ZqSA01HardwareBridge : public HardwareBridge
{
public:
    ZqSA01HardwareBridge(RobotController *rc, bool load_parameters_from_file);
    void runSpi();
    void initHardware();
    void loadConfigParam();
    void run();
    void runYesenseIMU();
    void runParamFileLoading();

    void abort(const std::string &reason);
    void abort(const char *reason);

private:
    VectorNavData _vectorNavData;
    VectorNavData _vectorNavData_tmp;
    lcm::LCM _spiLcm;
    spi_command_t sdu_imu_in_spi_command_t;
    std::thread _yesenseImuThread;

    yesense::YesenseDriver *yesense_imu;
    bool yesense_imu_init = false;

    bool _microstrainInit = false;
    bool _load_parameters_from_file;
    std::array<float, 1000> imu_res;
};

#endif // END of #ifdef linux
#endif // ZQ_HUMANOID_HARDWAREBRIDGE_H
