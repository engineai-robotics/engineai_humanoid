//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_RT_RC_INTERFACE_H
#define ZQ_HUMANOID_RT_RC_INTERFACE_H

class rc_control_settings
{
public:
    double mode;
    double p_des[2];         // (x, y) -1 ~ 1
    double height_variation; // -1 ~ 1
    double v_des[3];         // -1 ~ 1 * (scale 0.5 ~ 1.5)
    double rpy_des[3];       // -1 ~ 1
    double omega_des[3];     // -1 ~ 1
    double variable[3];
    double step_height;
    int gait_type = 1;
    int last_gait_type = 1;

    int imu_linvel_calibration_mode = 0;
    int last_imu_linvel_calibration_mode = 0; // used to record the pre button state of the trigger
    int last_vel_crossx = 0;
    int last_vel_crossy = 0;
    double cmd_vel_bias_x = 0;
    double cmd_vel_bias_y = 0;

    int imu_euler_angle_calibration_mode = 0;
    int last_imu_euler_angle_calibration_mode = 0; // used to record the pre button state of the trigger
    int last_crossx = 0;
    int last_crossy = 0;
    double pitch_bias = 0;
    double roll_bias = 0;

    bool bias_save_mode = false;
    bool bias_save_mode_pre = false;
};

namespace RC_mode
{
    constexpr int OFF = 0;
    constexpr int PASSIVE = 1;
    constexpr int STAND_UP = 2;
    constexpr int QP_STAND = 3;
    constexpr int BACKFLIP_PRE = 4;
    constexpr int BACKFLIP = 5;
    constexpr int VISION = 6;
    constexpr int BALANCE_STAND = 7;
    constexpr int LOCK_JOINT = 8;

    constexpr int LOCOMOTION = 11;
    constexpr int RECOVERY_STAND = 12;
    constexpr int FRONT_JUMP = 13;
    constexpr int STAND_DOWN = 15;

    constexpr int VMLOCOMOTION = 101;    // 测试阶段模式
    constexpr int VMWBCLOCOMOTION = 102; // 测试阶段模式

    // Experiment Mode
    constexpr int TWO_LEG_STANCE_PRE = 20;
    constexpr int TWO_LEG_STANCE = 21;

    constexpr int WAIT = 31;
};

void sbus_packet_complete();
void sbus_packet_complete_at9s();
void sbus_packet_complete_logitech();

void get_rc_control_settings(void *settings);

void *v_memcpy(void *dest, volatile void *src, size_t n);

float deadband(float command, float deadbandRegion, float minVal, float maxVal);
#endif // ZQ_HUMANOID_RT_RC_INTERFACE_H
