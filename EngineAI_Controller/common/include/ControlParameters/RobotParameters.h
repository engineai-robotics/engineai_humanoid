//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_ROBOTPARAMETERS_H
#define ZQ_HUMANOID_ROBOTPARAMETERS_H

#include "../ControlParameters/ControlParameters.h"

/*!
 * ControlParameters shared between all robot controllers
 */
class RobotControlParameters : public ControlParameters
{
public:
  /*!
   * Construct RobotControlParameters
   */
  RobotControlParameters()
      : ControlParameters("robot-parameters"),
        INIT_PARAMETER(IMU_acc_filter_ratio),
        INIT_PARAMETER(IMU_omega_filter_ratio),
        INIT_PARAMETER(foot_vel_filter_ratio),
        INIT_PARAMETER(use_joint_test_RL),
        INIT_PARAMETER(sin_wave_period),
        INIT_PARAMETER(leg_0_qdes),
        INIT_PARAMETER(leg_0_qddes),
        INIT_PARAMETER(leg_0_kp),
        INIT_PARAMETER(leg_0_kd),
        INIT_PARAMETER(leg_0_tauff),
        INIT_PARAMETER(leg_1_qdes),
        INIT_PARAMETER(leg_1_qddes),
        INIT_PARAMETER(leg_1_kp),
        INIT_PARAMETER(leg_1_kd),
        INIT_PARAMETER(leg_1_tauff),
        INIT_PARAMETER(leg_2_qdes),
        INIT_PARAMETER(leg_2_qddes),
        INIT_PARAMETER(leg_2_kp),
        INIT_PARAMETER(leg_2_kd),
        INIT_PARAMETER(leg_2_tauff),
        INIT_PARAMETER(leg_3_qdes),
        INIT_PARAMETER(leg_3_qddes),
        INIT_PARAMETER(leg_3_kp),
        INIT_PARAMETER(leg_3_kd),
        INIT_PARAMETER(leg_3_tauff),
        INIT_PARAMETER(controller_dt),
        INIT_PARAMETER(control_mode),
        INIT_PARAMETER(use_rc),
        INIT_PARAMETER(straight_stand_jpos),
        INIT_PARAMETER(squat_stand_jpos),
        INIT_PARAMETER(safety_damper_kd),
        INIT_PARAMETER(stand_joint_kp),
        INIT_PARAMETER(stand_joint_kd)
  {
  }
  DECLARE_PARAMETER(double, IMU_acc_filter_ratio)
  DECLARE_PARAMETER(double, IMU_omega_filter_ratio)
  DECLARE_PARAMETER(double, foot_vel_filter_ratio)
  DECLARE_PARAMETER(s64, use_joint_test_RL)
  DECLARE_PARAMETER(double, sin_wave_period)
  DECLARE_PARAMETER(Vec3<double>, leg_0_qdes)
  DECLARE_PARAMETER(Vec3<double>, leg_0_qddes)
  DECLARE_PARAMETER(Vec3<double>, leg_0_kp)
  DECLARE_PARAMETER(Vec3<double>, leg_0_kd)
  DECLARE_PARAMETER(Vec3<double>, leg_0_tauff)
  DECLARE_PARAMETER(Vec3<double>, leg_1_qdes)
  DECLARE_PARAMETER(Vec3<double>, leg_1_qddes)
  DECLARE_PARAMETER(Vec3<double>, leg_1_kp)
  DECLARE_PARAMETER(Vec3<double>, leg_1_kd)
  DECLARE_PARAMETER(Vec3<double>, leg_1_tauff)
  DECLARE_PARAMETER(Vec3<double>, leg_2_qdes)
  DECLARE_PARAMETER(Vec3<double>, leg_2_qddes)
  DECLARE_PARAMETER(Vec3<double>, leg_2_kp)
  DECLARE_PARAMETER(Vec3<double>, leg_2_kd)
  DECLARE_PARAMETER(Vec3<double>, leg_2_tauff)
  DECLARE_PARAMETER(Vec3<double>, leg_3_qdes)
  DECLARE_PARAMETER(Vec3<double>, leg_3_qddes)
  DECLARE_PARAMETER(Vec3<double>, leg_3_kp)
  DECLARE_PARAMETER(Vec3<double>, leg_3_kd)
  DECLARE_PARAMETER(Vec3<double>, leg_3_tauff)
  DECLARE_PARAMETER(double, controller_dt)
  DECLARE_PARAMETER(double, control_mode)
  DECLARE_PARAMETER(s64, use_rc);

  // RecoveryStand pitch joint positions
  DECLARE_PARAMETER(Vec3<double>, straight_stand_jpos);
  DECLARE_PARAMETER(Vec3<double>, squat_stand_jpos);
  DECLARE_PARAMETER(Vec3<double>, safety_damper_kd);
  DECLARE_PARAMETER(Vec3<double>, stand_joint_kp);
  DECLARE_PARAMETER(Vec3<double>, stand_joint_kd);
};
#endif // ZQ_HUMANOID_ROBOTPARAMETERS_H
