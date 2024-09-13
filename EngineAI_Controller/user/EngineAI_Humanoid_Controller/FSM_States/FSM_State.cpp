//
// Created by engineai on 2024/07/03.
//
/*============================= FSM State =============================*/
/**
 * FSM State base class
 */

#include "FSM_State.h"

/**
 * Constructor for the FSM State class.
 *
 * @param _controlFSMData holds all of the relevant control data
 * @param stateNameIn the enumerated state name
 * @param stateStringIn the string name of the current FSM state
 */
template <typename T>
FSM_State<T>::FSM_State(ControlFSMData<T> *_controlFSMData,
                        FSM_StateName stateNameIn, std::string stateStringIn)
    : _data(_controlFSMData),
      stateName(stateNameIn),
      stateString(stateStringIn)
{
    transitionData.zero();
    //    std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn
    //              << std::endl;

    _data->userParameters->Kp_body = Vec3<double>(500, 500, 100);
    _data->userParameters->Kd_body = Vec3<double>(10, 10, 10);
    _data->userParameters->Kp_foot = Vec3<double>(250, 250, 250);
    _data->userParameters->Kd_foot = Vec3<double>(10, 10, 10);
    _data->userParameters->Kp_ori = Vec3<double>(150, 150, 0);
    _data->userParameters->Kd_ori = Vec3<double>(80, 80, 0);
    _data->userParameters->Kp_joint = Vec3<double>(350, 350, 350);
    _data->userParameters->Kd_joint = Vec3<double>(2, 2, 2);

    _data->userParameters->cmpc_gait = 9;
    _data->userParameters->cmpc_x_drag = 3;
    _data->userParameters->cmpc_use_sparse = 0;
    _data->userParameters->cmpc_bonus_swing = 0;
    _data->userParameters->jcqp_alpha = 1.5;
    _data->userParameters->jcqp_max_iter = 10000;
    _data->userParameters->jcqp_rho = 1e-07;
    _data->userParameters->jcqp_sigma = 1e-08;
    _data->userParameters->jcqp_terminate = 0.1;
    _data->userParameters->use_jcqp = 0;
    _data->userParameters->use_wbc = 0; // 1;

    _data->userParameters->Swing_Kp_cartesian = Vec3<double>(350, 350, 75);
    _data->userParameters->Swing_Kd_cartesian = Vec3<double>(5.5, 5.5, 5.5);
    _data->userParameters->Swing_Kp_joint = Vec3<double>(0, 0, 0);
    _data->userParameters->Swing_Kd_joint = Vec3<double>(0.2, 0.2, 0.2);
    _data->userParameters->Swing_step_offset = Vec3<double>(0, 0.05, -0.003);
    _data->userParameters->Swing_traj_height = 0.07;
    _data->userParameters->Swing_use_tau_ff = 0;

    _data->userParameters->gait_type = 4;
    _data->userParameters->gait_period_time = 0.5;
    _data->userParameters->gait_switching_phase = 0.5;
    _data->userParameters->gait_override = 4;
    _data->userParameters->gait_max_leg_angle = 15;
    _data->userParameters->gait_max_stance_time = 0.25;
    _data->userParameters->gait_min_stance_time = 0.1;
}

/**
 * Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param qDes desired joint position
 * @param dqDes desired joint velocity
 */
template <typename T>
void FSM_State<T>::jointPDControl(
    int leg, Vec3<T> qDes, Vec3<T> qdDes)
{

    if (_data->_humanoid_biped->_robotType == RobotType::ZQ_Biped_SA01)
    {
        kpMat << 300, 0, 0, 0, 300, 0, 0, 0, 300;
        kdMat << 3, 0, 0, 0, 3, 0, 0, 0, 3;
    }
    else if (_data->_humanoid_biped->_robotType == RobotType::ZQ_Biped_SA01P)
    {
        kpMat << 120, 0, 0, 0, 120, 0, 0, 0, 120;
        kdMat << 4, 0, 0, 0, 4, 0, 0, 0, 4;
    }

    _data->_legController->commands[leg].kpJoint = kpMat;
    _data->_legController->commands[leg].kdJoint = kdMat;

    _data->_legController->commands[leg].qDes = qDes;
    _data->_legController->commands[leg].qdDes = qdDes;
}
template <typename T>
void FSM_State<T>::jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes, Mat3<float> kp, Mat3<float> kd)
{
    kpMat = kp;
    kdMat = kd;
    _data->_legController->commands[leg].kpJoint = kpMat;
    _data->_legController->commands[leg].kdJoint = kdMat;

    _data->_legController->commands[leg].qDes = qDes;
    _data->_legController->commands[leg].qdDes = qdDes;
}

/**
 * Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param pDes desired foot position
 * @param vDes desired foot velocity
 * @param kp_cartesian P gains
 * @param kd_cartesian D gains
 */
template <typename T>
void FSM_State<T>::cartesianImpedanceControl(int leg, Vec3<T> pDes,
                                             Vec3<T> vDes,
                                             Vec3<double> kp_cartesian,
                                             Vec3<double> kd_cartesian)
{
    _data->_legController->commands[leg].pDes = pDes;
    // Create the cartesian P gain matrix
    kpMat << kp_cartesian[0], 0, 0, 0,
        kp_cartesian[1], 0, 0, 0,
        kp_cartesian[2];
    _data->_legController->commands[leg].kpCartesian = kpMat;

    _data->_legController->commands[leg].vDes = vDes;
    // Create the cartesian D gain matrix
    kdMat << kd_cartesian[0], 0, 0, 0, kd_cartesian[1], 0, 0, 0, kd_cartesian[2];
    _data->_legController->commands[leg].kdCartesian = kdMat;
}

/**
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
template <typename T>
void FSM_State<T>::turnOnAllSafetyChecks()
{
    // Pre controls safety checks
    checkSafeOrientation = true; // check roll and pitch

    // Post control safety checks
    checkPDesFoot = true;         // do not command footsetps too far
    checkForceFeedForward = true; // do not command huge forces
    checkLegSingularity = true;   // do not let leg
}

/**
 *
 */
template <typename T>
void FSM_State<T>::turnOffAllSafetyChecks()
{
    // Pre controls safety checks
    checkSafeOrientation = false; // check roll and pitch

    // Post control safety checks
    checkPDesFoot = false;         // do not command footsetps too far
    checkForceFeedForward = false; // do not command huge forces
    checkLegSingularity = false;   // do not let leg
}

// template class FSM_State<double>;
template class FSM_State<float>;
