//
// Created by engineai on 2024/07/03.
//
/*! @file LegController.cpp
 *  @brief Common Leg Control Interface
 *
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */
#include "Controllers/LegController.h"
#include "config.h"
#include <iostream>
#include <fstream>

/*!
 * Zero the leg command so the leg will not output torque
 */
template <typename T>
void LegControllerCommand<T>::zero()
{
    tauFeedForward = Vec3<T>::Zero();
    forceFeedForward = Vec3<T>::Zero();
    qDes = Vec3<T>::Zero();
    qdDes = Vec3<T>::Zero();
    pDes = Vec3<T>::Zero();
    vDes = Vec3<T>::Zero();
    kpCartesian = Mat3<T>::Zero();
    kdCartesian = Mat3<T>::Zero();
    kpJoint = Mat3<T>::Zero();
    kdJoint = Mat3<T>::Zero();
}

/*!
 * Zero the leg data
 */
template <typename T>
void LegControllerData<T>::zero()
{
    q = Vec3<T>::Zero();
    qd = Vec3<T>::Zero();
    p = Vec3<T>::Zero();
    v = Vec3<T>::Zero();
    J = Mat3<T>::Zero();
    tauEstimate = Vec3<T>::Zero();
    q_foot_p = 0.;
    qd_foot_p = 0.;
    q_foot_r = 0.;
    qd_foot_r = 0.;
    q_hip = 0.;
    qd_hip = 0.;
}

void LegControllerDataBiped::zero()
{
    q = Vec12<float>::Zero();
    qd = Vec12<float>::Zero();
    tau_state = Vec12<float>::Zero();

    q_des = Vec12<float>::Zero();
    qd_des = Vec12<float>::Zero();
    tau_ff = Vec12<float>::Zero();
    kp = Vec12<float>::Zero();
    kd = Vec12<float>::Zero();
}

/*!
 * Zero all leg commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
template <typename T>
void LegController<T>::zeroCommand()
{
    for (auto &cmd : commands)
    {
        cmd.zero();
    }
    _legsEnabled = false;
}

/*!
 * Update the "leg data" from a SPIne board message
 */
template <typename T>
void LegController<T>::updateData(motor_data_t *jointdatas)
{
    for (int i = 0; i < 12; i++)
    {
        leg_control_data_biped.q[i] = jointdatas->q[i];
        leg_control_data_biped.qd[i] = jointdatas->qd[i];
        leg_control_data_biped.tau_state[i] = jointdatas->tau[i];
    }
}

template <typename T>
void LegController<T>::updateCommand(motor_command_t *jointcommands)
{

    for (int i = 0; i < 12; i++)
    {
        jointcommands->q_des[i] = leg_control_data_biped.q_des[i];
        jointcommands->qd_des[i] = leg_control_data_biped.qd_des[i];
        jointcommands->tau_ff[i] = leg_control_data_biped.tau_ff[i];
        jointcommands->kp[i] = leg_control_data_biped.kp[i];
        jointcommands->kd[i] = leg_control_data_biped.kd[i];
    }
}

/*!
 * Set LCM debug data from leg commands and data
 */
template <typename T>
void LegController<T>::setLcm(leg_control_data_lcmt *lcmData, leg_control_command_lcmt *lcmCommand)
{
    for (int joint = 0; joint < 12; joint++)
    {
        lcmData->q[joint] = leg_control_data_biped.q[joint];
        lcmData->qd[joint] = leg_control_data_biped.qd[joint];
        lcmData->tau_est[joint] = leg_control_data_biped.tau_state[joint];

        lcmCommand->q_des[joint] = leg_control_data_biped.q_des[joint];
        lcmCommand->qd_des[joint] = leg_control_data_biped.qd_des[joint];
        lcmCommand->tau_ff[joint] = leg_control_data_biped.tau_ff[joint];
        lcmCommand->kp_joint[joint] = leg_control_data_biped.kp[joint];
        lcmCommand->kd_joint[joint] = leg_control_data_biped.kd[joint];
    }
}

template <typename T>
void LegController<T>::computeGravityTorqueForSwingLeg(RobotConstructor<T> *biped, Vec3<T> &q,
                                                       Vec3<T> *gravityTorque, Mat3<T> bodyRotMat, int leg, bool is_swing)
{
    T l1 = 0.1; // biped._abadLinkLength;
    T l2 = biped->_hipLinkLength - 0.1;
    T l3 = biped->_kneeLinkLength;
    T l4 = biped->_kneeLinkY_offset;
    T sideSign = biped->getSideSign(leg);
    Vec3<T> g_acc(0.0, 0.0, -9.81); // gravity acc in world frame

    Vec3<T> pos_hip_COM_local = biped->_hipInertia.getCOM();
    Vec3<T> pos_knee_COM_local = biped->_kneeInertia.getCOM();
    Vec3<T> pos_knee_joint_local(0.0, 0.0, -l2);
    T mass_hip = biped->_hipInertia.getMass();
    T mass_knee = biped->_kneeInertia.getMass();

    // calculate the COM position of each link w.r.t. leg coordinate frame
    Mat3<T> rot_abad = ori::coordinateRotation(CoordinateAxis::X, -q(0));
    Mat3<T> rot_hip = ori::coordinateRotation(CoordinateAxis::Y, -q(1));
    Mat3<T> rot_knee = ori::coordinateRotation(CoordinateAxis::Y, -q(2));

    Vec3<T> pos_hip_COM_to_hip_axis = rot_abad * rot_hip * pos_hip_COM_local;                                       // used to calculate gravity tau on hip joint caused by hip link
    Vec3<T> pos_knee_COM_to_hip_axis = rot_abad * rot_hip * (pos_knee_joint_local + rot_knee * pos_knee_COM_local); // used to calculate gravity tau on hip joint caused by knee link
    Vec3<T> pos_knee_COM_to_knee_axis = rot_abad * rot_hip * rot_knee * pos_knee_COM_local;                         // used to calculate gravity tau on knee joint caused by knee link

    // std::cout << "leg: " << leg << " ,COM Pos in leg frame: " << std::endl;
    // std::cout << pos_hip_COM_to_hip_axis.transpose() << std::endl;
    // std::cout << pos_knee_COM_to_hip_axis.transpose() << std::endl;
    // std::cout << pos_knee_COM_to_knee_axis.transpose() << std::endl;
    Vec3<T> hip_COM_to_hip_in_world = bodyRotMat.transpose() * pos_hip_COM_to_hip_axis;
    Vec3<T> knee_COM_to_hip_in_world = bodyRotMat.transpose() * pos_knee_COM_to_hip_axis;
    Vec3<T> knee_COM_to_knee_in_world = bodyRotMat.transpose() * pos_knee_COM_to_knee_axis;

    // std::cout << "leg: " << leg << " ,COM Pos in world frame: " << std::endl;
    // std::cout << hip_COM_to_hip_in_world.transpose() << std::endl;
    // std::cout << knee_COM_to_hip_in_world.transpose() << std::endl;
    // std::cout << knee_COM_to_knee_in_world.transpose() << std::endl;

    // calculate the gravity acc w.r.t. leg coordinate frame
    g_acc = bodyRotMat * g_acc;

    // calculate the compensation torque of gravity
    // if (is_swing)
    // {
    Vec3<T> hip_torque_vec_global = mass_hip * crossMatrix(g_acc) * pos_hip_COM_to_hip_axis + mass_knee * crossMatrix(g_acc) * pos_knee_COM_to_hip_axis;
    Vec3<T> knee_torque_vec_global = mass_knee * crossMatrix(g_acc) * pos_knee_COM_to_knee_axis;
    Vec3<T> hip_torque_vec_local = rot_hip.transpose() * rot_abad.transpose() * hip_torque_vec_global;
    Vec3<T> knee_torque_vec_local = rot_knee.transpose() * rot_hip.transpose() * rot_abad.transpose() * knee_torque_vec_global;

    gravityTorque->operator()(0) = 0.0;
    gravityTorque->operator()(1) = hip_torque_vec_local[1];
    gravityTorque->operator()(2) = knee_torque_vec_local[1];
    // }
    // else
    // {
    //     gravityTorque->operator()(0) = 0;
    //     gravityTorque->operator()(1) = 0;
    //     gravityTorque->operator()(2) = 0;
    // }
}

template <typename T>
void LegController<T>::InverseKinematicForLeg(RobotConstructor<T> *biped, Vec3<T> &q,
                                              Vec3<T> *p, int leg)
{
    T l1 = 0.1; // biped._abadLinkLength;
    T l2 = biped->_hipLinkLength - 0.1;
    T l3 = biped->_kneeLinkLength;

    T s1 = std::sin(q(0));
    T s2 = std::sin(q(1));
    T s3 = std::sin(q(2));

    T c1 = std::cos(q(0));
    T c2 = std::cos(q(1));
    T c3 = std::cos(q(2));

    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;

    T pos_x = p->operator()(0);
    T pos_y = p->operator()(1);
    T pos_z = p->operator()(2);

    q.setZero();

    if (fabs(pos_z) < 1e-6)
        pos_z = 0.0;

    // std::cout << "pos: " << pos_x << ", " << pos_y << ", " << pos_z << std::endl;
    Vec2<T> thetaHip(0.0, 0.0);

    if (p)
    {

        q[0] = std::atan2(pos_y, -pos_z);
        // std::cout << "q[0] ori: " << q[0] << std::endl;

        if (fabs(pos_y) < 0.000001 && fabs(pos_z) < 0.000001)
        {
            std::cout << "end point is on x axis" << std::endl;
            q[0] = 0.0;
        }

        // hip and knee joints have multi-solutions
        // method 1: geometrical method
        T hight2YOZ = pos_x;
        T proj_length = 0;

        if (fabs(q[0] - (-M_PI / 2)) < 0.000001 || fabs(q[0] - M_PI / 2) < 0.000001)
        {
            std::cout << "reaching boundry" << std::endl;
            proj_length = pos_y / std::sin(q[0]) - l1;
            std::cout << "proj_length ori_1: " << pos_y / std::sin(q[0]) - l1 << std::endl;
        }
        else
        {
            proj_length = -pos_z / std::cos(q[0]) - l1; // projection of hip2end vector in YOZ plane
            // std::cout << "proj_length ori_2: " << -pos_z / std::cos(q[0]) - l1 << std::endl;
        }

        T middle_angle = std::atan2(hight2YOZ, proj_length); // angle between hip2end vector and YOZ plane

        // std::cout << "middle_angle ori: " << middle_angle << std::endl;

        T hip2endLength = sqrt(pow(proj_length, 2) + pow(hight2YOZ, 2));

        T delta_thetaHip = 0;
        if (fabs(hip2endLength - l2 - l3) > 0.000001)
            delta_thetaHip = std::acos((pow(l2, 2) + pow(hip2endLength, 2) - pow(l3, 2)) / (2 * l2 * hip2endLength)); // angle between l2 link and hip2end vector

        thetaHip[0] = middle_angle + delta_thetaHip; // first one  of 2 solutions
        thetaHip[1] = middle_angle - delta_thetaHip; // second  one  of 2 solutions

        q[1] = std::max(thetaHip[0], thetaHip[1]); // choose the maximum one for the configuration

        if (fabs(hip2endLength - l2 - l3) > 0.000001)
            q[2] = std::acos((pow(l2, 2) + pow(l3, 2) - pow(hip2endLength, 2)) / (2 * l2 * l3)) - M_PI;
    }
}

template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;

template struct LegControllerData<double>;
template struct LegControllerData<float>;

template class LegController<double>;
template class LegController<float>;

/*!
 * Compute the position of the foot and its Jacobian.  This is done in the local
 * leg coordinate system. If J/p are NULL, the calculation will be skipped.
 */
template <typename T>
void computeLegJacobianAndPosition(RobotConstructor<T> &biped, Vec3<T> &q, Mat3<T> *J,
                                   Vec3<T> *p, int leg)
{
    T l1 = biped._abadLinkLength;
    T l2 = biped._hipLinkLength;
    T l3 = biped._kneeLinkLength;
    T l4 = biped._kneeLinkY_offset;
    T sideSign = biped.getSideSign(leg);

    T s1 = std::sin(q(0));
    T s2 = std::sin(q(1));
    T s3 = std::sin(q(2));

    T c1 = std::cos(q(0));
    T c2 = std::cos(q(1));
    T c3 = std::cos(q(2));

    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;

    if (J)
    {
        J->operator()(0, 0) = 0;
        J->operator()(0, 1) = l3 * c23 + l2 * c2;
        J->operator()(0, 2) = l3 * c23;
        //		J->operator()(0, 1) = -l3 * c23 - l2 * c2;
        //		J->operator()(0, 2) = -l3 * c23;
        J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1;
        J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
        J->operator()(1, 2) = -l3 * s1 * s23;
        J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1;
        J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
        J->operator()(2, 2) = l3 * c1 * s23;
    }

    if (p)
    {
        p->operator()(0) = l3 * s23 + l2 * s2;
        //        p->operator()(0) = -l3 * s23 - l2 * s2;
        p->operator()(1) = (l1 + l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
        p->operator()(2) = (l1 + l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
    }
}

template <typename T>
void computeLegJacobianAndPositionModify(RobotConstructor<T> &biped, Vec3<T> &q, Mat3<T> *J,
                                         Vec3<T> *p, int leg)
{
    T l1 = 0.1; // biped._abadLinkLength;
    T l2 = biped._hipLinkLength - 0.1;
    T l3 = biped._kneeLinkLength;
    T l4 = biped._kneeLinkY_offset;
    T sideSign = biped.getSideSign(leg);

    T s1 = std::sin(q(0));
    T s2 = std::sin(q(1));
    T s3 = std::sin(q(2));

    T c1 = std::cos(q(0));
    T c2 = std::cos(q(1));
    T c3 = std::cos(q(2));

    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;

    if (J)
    {
        J->operator()(0, 0) = 0;
        J->operator()(0, 1) = l2 * c2 + l3 * c23;
        J->operator()(0, 2) = l3 * c23;

        J->operator()(1, 0) = (l1 + l2 * c2 + l3 * c23) * c1;
        J->operator()(1, 1) = -(l2 * s2 + l3 * s23) * s1;
        J->operator()(1, 2) = -l3 * s23 * s1;

        J->operator()(2, 0) = (l1 + l2 * c2 + l3 * c23) * s1;
        J->operator()(2, 1) = (l2 * s2 + l3 * s23) * c1;
        J->operator()(2, 2) = l3 * s23 * c1;
    }

    if (p)
    {
        p->operator()(0) = l2 * s2 + l3 * s23;
        p->operator()(1) = (l1 + l2 * c2 + l3 * c23) * s1;
        p->operator()(2) = -(l1 + l2 * c2 + l3 * c23) * c1;
    }
}

template void computeLegJacobianAndPosition<double>(RobotConstructor<double> &biped,
                                                    Vec3<double> &q,
                                                    Mat3<double> *J,
                                                    Vec3<double> *p, int leg);
template void computeLegJacobianAndPosition<float>(RobotConstructor<float> &biped,
                                                   Vec3<float> &q,
                                                   Mat3<float> *J,
                                                   Vec3<float> *p, int leg);
