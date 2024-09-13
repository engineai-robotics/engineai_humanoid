//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_LEGCONTROLLER_H
#define ZQ_HUMANOID_LEGCONTROLLER_H

#include "../cppTypes.h"
#include "../../../lcm-types/cpp/leg_control_command_lcmt.hpp"
#include "../../../lcm-types/cpp/leg_control_data_lcmt.hpp"
#include "../../../lcm-types/cpp/spi_data_t.hpp"
#include "../../../lcm-types/cpp/spi_command_t.hpp"
#include "../Dynamics/RobotConstructor.h"
#include "../SimUtilities/SpineBoard.h"
#include "../SimUtilities/ti_boardcontrol.h"
#include "../config.h"
#include <lcm/lcm-cpp.hpp>
#include "../ControlParameters/RobotParameters.h"
#include "motor.h"

/*!
 * Data sent from the control algorithm to the legs.
 */
template <typename T>
struct LegControllerCommand
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerCommand()
    {
        zero();
    }

    void zero();

    Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
    float qDes_hip, qDes_foot_p, qDes_foot_r, qdDes_hip, qdDes_foot_p, qdDes_foot_r;
    Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
    float kp_hip, kp_foot, kd_hip, kd_foot;
    float tauFeedForward_hip, tauFeedForward_foot;
};

/*!
 * Data returned from the legs to the control code.
 */
template <typename T>
struct LegControllerData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerData()
    {
        zero();
    }

    void setBiped(RobotConstructor<T> &biped)
    {
        humanoid_biped = &biped;
    }

    void zero();

    Vec3<T> q, qd, p, v;
    float q_hip, q_foot_r, q_foot_p;
    float qd_hip, qd_foot_r, qd_foot_p;
    Mat3<T> J;
    Vec3<T> tauEstimate;
    float tauEstimate_hip, tauEstimate_foot_p, tauEstimate_foot_r;
    double footTouchDown;
    double footForce3D[3];
    RobotConstructor<T> *humanoid_biped;

    Vec3<T> p_error;
};

struct LegControllerDataBiped
{

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerDataBiped()
    {
        zero();
    }

    void zero();

    Vec12<float> q;
    Vec12<float> qd;
    Vec12<float> tau_state;

    Vec12<float> q_des;
    Vec12<float> qd_des;
    Vec12<float> tau_ff;
    Vec12<float> kp;
    Vec12<float> kd;
};
/*!
 * Controller for biped robot
 */
template <typename T>
class LegController
{
public:
    LegController(RobotConstructor<T> &biped, RobotControlParameters *parameters) : _humanoid_biped(biped),
                                                                                    _parameters(parameters)
    {
        for (auto &data : datas)
        {
            data.setBiped(_humanoid_biped);
        }
    }

    ~LegController()
    {
    }

    void zeroCommand();
    void updateData(motor_data_t *jointdatas);
    void updateCommand(motor_command_t *jointcommands);
    void InverseKinematicForLeg(RobotConstructor<T> *biped, Vec3<T> &q,
                                Vec3<T> *p, int leg);
    void computeGravityTorqueForSwingLeg(RobotConstructor<T> *biped, Vec3<T> &q,
                                         Vec3<T> *gravityTorque, Mat3<T> bodyRotMat, int leg, bool is_swing);
    void setEnabled(bool enabled)
    {
        _legsEnabled = enabled;
    };
    void setLcm(leg_control_data_lcmt *data, leg_control_command_lcmt *command);

    LegControllerCommand<T> commands[2];
    LegControllerData<T> datas[2];
    RobotConstructor<T> &_humanoid_biped;
    RobotControlParameters *_parameters = nullptr;

    bool _legsEnabled = false;

    spi_command_t spi_cmd_in_leg_controller;
    leg_control_command_lcmt leg_controller_cart_pos;
    leg_control_command_lcmt footforce_tau_ff;

    LegControllerDataBiped leg_control_data_biped;

    bool check_j_vel_fail = false;

    T q_send_test = -12.5;
    // Timer comm_time;
    int loop_count = 0;
    int sign = 1;
    struct timespec t_spec;
    bool first_comm = true;

    Vec3<T> right_leg_qd_pre;
    Vec3<T> left_leg_qd_pre;

    bool *motor_enable_flag = nullptr;
    bool *motor_disable_flag = nullptr;
};

template <typename T>
void computeLegJacobianAndPosition(RobotConstructor<T> &biped, Vec3<T> &q, Mat3<T> *J,
                                   Vec3<T> *p, int leg);

template <typename T>
void computeLegJacobianAndPositionModify(RobotConstructor<T> &biped, Vec3<T> &q, Mat3<T> *J,
                                         Vec3<T> *p, int leg);

#endif // ZQ_HUMANOID_LEGCONTROLLER_H
