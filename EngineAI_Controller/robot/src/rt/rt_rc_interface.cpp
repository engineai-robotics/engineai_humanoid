#include <pthread.h>
#include <rt/rt_rc_interface.h>
#include "Utilities/EdgeTrigger.h"
#include <string.h> // memcpy
#include <stdio.h>
#include <rt/rt_sbus.h>
#include <iostream>

#define BUTTERN_PRESS_THRESHOULD 0.5

static pthread_mutex_t lcm_get_set_mutex =
    PTHREAD_MUTEX_INITIALIZER; /**< mutex to protect gui settings coming over
                         LCM */

// Controller Settings
rc_control_settings rc_control;
int selected_mode_logitech = 0;
/* ------------------------- HANDLERS ------------------------- */

// Controller Settings
void get_rc_control_settings(void *settings)
{
    pthread_mutex_lock(&lcm_get_set_mutex);
    v_memcpy(settings, &rc_control, sizeof(rc_control_settings));
    pthread_mutex_unlock(&lcm_get_set_mutex);
}

// void get_rc_channels(void *settings) {
// pthread_mutex_lock(&lcm_get_set_mutex);
// v_memcpy(settings, &rc_channels, sizeof(rc_channels));
// pthread_mutex_unlock(&lcm_get_set_mutex);
// }

EdgeTrigger<int> mode_edge_trigger(0);
EdgeTrigger<TaranisSwitchState> backflip_prep_edge_trigger(SWITCH_UP);
EdgeTrigger<TaranisSwitchState> experiment_prep_edge_trigger(SWITCH_UP);
TaranisSwitchState initial_mode_go_switch = SWITCH_DOWN;

void sbus_packet_complete_at9s()
{
    AT9s_data data;
    update_taranis_at9s(&data);
    float v_scale = 1.5;
    float w_scale = 1.5 * v_scale;

    auto estop_switch = data.SWE; // 开关
    auto QP_Locomotion_switch = data.SWA;
    // auto Locomotion_switch=data.SWB;

    auto left_select = data.SWC;
    auto right_select = data.SWD;

    auto normal_jump_flip_switch = data.SWG;
    auto roll_show = 0.0;               // data.varB*1.1;
    auto step_height = data.varB + 1.0; // varB 范围是-1～1
    int selected_mode = 0;
    // printf("at9s in\n");
    switch (estop_switch)
    {
    case AT9S_TRI_UP: // 停止
        selected_mode = RC_mode::OFF;
        break;

    case AT9S_TRI_MIDDLE:
        selected_mode = RC_mode::RECOVERY_STAND;
        break;

    case AT9S_TRI_DOWN:
        // selected_mode = RC_mode::LOCOMOTION; // 默认模式
        if (normal_jump_flip_switch == AT9S_TRI_MIDDLE) // 正常运动模式
        {

            if (QP_Locomotion_switch == AT9S_BOOL_UP)
            {
                selected_mode = RC_mode::LOCOMOTION;

                // Deadband
                data.left_stick_x = deadband(data.left_stick_x, 0.1, -1., 1.);
                data.left_stick_y = deadband(data.left_stick_y, 0.1, -1., 1.);
                data.right_stick_x = deadband(data.right_stick_x, 0.1, -1., 1.);
                data.right_stick_y = deadband(data.right_stick_y, 0.1, -1., 1.);

                int gait_id = 9;
                if (right_select == AT9S_BOOL_UP) // 低速运动模式
                {
                    if (left_select == AT9S_TRI_UP)
                        gait_id = 9; // trot
                    else if (left_select == AT9S_TRI_MIDDLE)
                        gait_id = 3;                       // slow trot
                    else if (left_select == AT9S_TRI_DOWN) // walk
                        gait_id = 6;
                }
                else if (right_select == AT9S_BOOL_DOWN) // 高速运动模式
                {
                    if (left_select == AT9S_TRI_UP)
                        gait_id = 5; // flying trot
                    else if (left_select == AT9S_TRI_MIDDLE)
                        gait_id = 1; // bound
                    else if (left_select == AT9S_TRI_DOWN)
                        gait_id = 2; // pronk
                }

                rc_control.variable[0] = gait_id;
                rc_control.v_des[0] = data.right_stick_x > 0 ? v_scale * data.right_stick_x : v_scale * data.right_stick_x;
                rc_control.v_des[1] = -1.0 * data.right_stick_y; // -v_scale * data.right_stick_y;
                rc_control.v_des[2] = 0;

                rc_control.omega_des[0] = 0;
                rc_control.omega_des[1] = data.left_stick_x; // 0;//pitch
                rc_control.omega_des[2] = w_scale * data.left_stick_y;

                rc_control.rpy_des[0] = roll_show;
                rc_control.step_height = step_height * 0.1;
            }
            else if (QP_Locomotion_switch == AT9S_BOOL_DOWN)
            {
                selected_mode = RC_mode::QP_STAND;
                rc_control.rpy_des[0] = data.left_stick_y;
                rc_control.rpy_des[1] = data.left_stick_x;
                rc_control.rpy_des[2] = data.right_stick_y;

                rc_control.height_variation = data.right_stick_x;

                rc_control.omega_des[0] = 0;
                rc_control.omega_des[1] = 0;
                rc_control.omega_des[2] = 0;
            }
        }
        else if (normal_jump_flip_switch == AT9S_TRI_UP)
        {
            //                selected_mode = RC_mode::BACKFLIP;
            selected_mode = RC_mode::FRONT_JUMP;
        }
        else if (normal_jump_flip_switch == AT9S_TRI_DOWN)
        {
            //                selected_mode = RC_mode::FRONT_JUMP;

            ////                selected_mode = RC_mode::VMLOCOMOTION;
            selected_mode = RC_mode::VMWBCLOCOMOTION;

            data.left_stick_x = deadband(data.left_stick_x, 0.1, -1., 1.);
            data.left_stick_y = deadband(data.left_stick_y, 0.1, -1., 1.);
            data.right_stick_x = deadband(data.right_stick_x, 0.1, -1., 1.);
            data.right_stick_y = deadband(data.right_stick_y, 0.1, -1., 1.);
            rc_control.v_des[0] = data.right_stick_x > 0 ? v_scale * data.right_stick_x : v_scale * data.right_stick_x;
            rc_control.v_des[1] = -1.0 * data.right_stick_y; // -v_scale * data.right_stick_y;
            rc_control.v_des[2] = 0;

            rc_control.omega_des[0] = 0;
            rc_control.omega_des[1] = data.left_stick_x; // 0;//pitch
            rc_control.omega_des[2] = w_scale * data.left_stick_y;

            rc_control.rpy_des[0] = roll_show;
            rc_control.rpy_des[1] = data.left_stick_x; // 0;//pitch
            rc_control.step_height = step_height * 0.1;
            int gait_id = 9;
            if (right_select == AT9S_BOOL_UP) // 低速运动模式
            {
                if (left_select == AT9S_TRI_UP)
                    gait_id = 9; // trot
                else if (left_select == AT9S_TRI_MIDDLE)
                    gait_id = 3;                       // slow trot
                else if (left_select == AT9S_TRI_DOWN) // walk
                    gait_id = 6;
            }
            else if (right_select == AT9S_BOOL_DOWN) // 高速运动模式
            {
                if (left_select == AT9S_TRI_UP)
                    gait_id = 5; // flying trot
                else if (left_select == AT9S_TRI_MIDDLE)
                    gait_id = 1; // bound
                else if (left_select == AT9S_TRI_DOWN)
                    gait_id = 2; // pronk
            }
            rc_control.variable[0] = gait_id;
            //                printf("id: %d\n",gait_id);
        }

        break;
    }
    rc_control.mode = selected_mode;
}
void sbus_packet_complete()
{
    Taranis_X7_data data;
    update_taranis_x7(&data);

    float v_scale = data.knobs[0] * 1.5f + 2.0f; // from 0.5 to 3.5
    float w_scale = 2. * v_scale;                // from 1.0 to 7.0
    // printf("v scale: %f\n", v_scale);

    auto estop_switch = data.right_lower_right_switch;        //  最右侧开关控制EStop, 向上表示关闭，中间表示recover，下边表示run
    auto mode_selection_switch = data.left_lower_left_switch; // 最左侧开关，控制运动模式，是backflip mpc 还是其他
    auto mode_go_switch = data.left_upper_switch;

    auto left_select = data.left_lower_right_switch;  // 这两个开关(3中模式)实现9种步态种类选择
    auto right_select = data.right_lower_left_switch; // 这两个开关(3中模式)实现9种步态种类选择

    int selected_mode = 0;

    switch (estop_switch)
    {

    case SWITCH_UP: // ESTOP
        selected_mode = RC_mode::OFF;
        break;

    case SWITCH_MIDDLE: // recover
        selected_mode = RC_mode::RECOVERY_STAND;
        break;

    case SWITCH_DOWN:                        // run
        selected_mode = RC_mode::LOCOMOTION; // locomotion by default

        // stand mode
        if (left_select == SWITCH_UP && right_select == SWITCH_UP)
        {
            selected_mode = RC_mode::QP_STAND;
        }

        if (backflip_prep_edge_trigger.trigger(mode_selection_switch) && mode_selection_switch == SWITCH_MIDDLE)
        {
            initial_mode_go_switch = mode_go_switch;
        }

        // Experiment mode (two leg stance, vision, ...)
        if (experiment_prep_edge_trigger.trigger(mode_selection_switch) && mode_selection_switch == SWITCH_DOWN)
        {
            initial_mode_go_switch = mode_go_switch;
        }

        // backflip
        if (mode_selection_switch == SWITCH_MIDDLE)
        {
            selected_mode = RC_mode::BACKFLIP_PRE;

            if (mode_go_switch == SWITCH_DOWN && initial_mode_go_switch != SWITCH_DOWN)
            {
                selected_mode = RC_mode::BACKFLIP;
            }
            else if (mode_go_switch == SWITCH_UP)
            {
                initial_mode_go_switch = SWITCH_UP;
            }
        } // Experiment Mode
        else if (mode_selection_switch == SWITCH_DOWN)
        {
            int mode_id = left_select * 3 + right_select;

            if (mode_id == 0)
            { // Two leg stance
                selected_mode = RC_mode::TWO_LEG_STANCE_PRE;
                if (mode_go_switch == SWITCH_DOWN && initial_mode_go_switch != SWITCH_DOWN)
                {
                    selected_mode = RC_mode::TWO_LEG_STANCE;
                }
                else if (mode_go_switch == SWITCH_UP)
                {
                    initial_mode_go_switch = SWITCH_UP;
                }
            }
            else if (mode_id == 1)
            { // Vision
                selected_mode = RC_mode::VISION;
            }
        }

        // gait selection
        int mode_id = left_select * 3 + right_select;

        constexpr int gait_table[9] = {
            0, // stand
            0, // trot
            1, // bounding
            2, // pronking
            3, // gallop
            5, // trot run
            6, // walk};
            7, // walk2?
            8, // pace
        };

        // Deadband
        for (int i(0); i < 2; ++i)
        {
            data.left_stick[i] = deadband(data.left_stick[i], 0.1, -1., 1.);
            data.right_stick[i] = deadband(data.right_stick[i], 0.1, -1., 1.);
        }

        if (selected_mode == RC_mode::LOCOMOTION || selected_mode == RC_mode::VISION)
        {
            rc_control.variable[0] = gait_table[mode_id];
            // rc_control.v_des[0] = v_scale * data.left_stick[1] * 0.5;
            // rc_control.v_des[1] = v_scale * data.left_stick[0] * -1.;
            rc_control.v_des[0] = v_scale * data.left_stick[1];
            rc_control.v_des[1] = -v_scale * data.left_stick[0];
            rc_control.v_des[2] = 0;

            rc_control.height_variation = data.knobs[1];
            // rc_control.p_des[2] = 0.27 + 0.08 * data.knobs[1]; // todo?

            rc_control.omega_des[0] = 0;
            rc_control.omega_des[1] = 0;
            rc_control.omega_des[2] = w_scale * data.right_stick[0];
            // rc_control.omega_des[2] = -v_scale * data.right_stick[0];
        }
        else if (selected_mode == RC_mode::QP_STAND || selected_mode == RC_mode::TWO_LEG_STANCE)
        {
            // rc_control.rpy_des[0] = data.left_stick[0] * 1.4;
            // rc_control.rpy_des[1] = data.right_stick[1] * 0.46;
            rc_control.rpy_des[0] = data.left_stick[0];
            rc_control.rpy_des[1] = data.right_stick[1];
            rc_control.rpy_des[2] = data.right_stick[0];

            rc_control.height_variation = data.left_stick[1];

            rc_control.omega_des[0] = 0;
            rc_control.omega_des[1] = 0;
            rc_control.omega_des[2] = 0;
            // rc_control.p_des[1] = -0.667 * rc_control.rpy_des[0];
            // rc_control.p_des[2] = data.left_stick[1] * .12;
        }
        break;
    }
    // 模式切换时候将模式给定到rc_control.mode上
    bool trigger = mode_edge_trigger.trigger(selected_mode);
    if (trigger || selected_mode == RC_mode::OFF || selected_mode == RC_mode::RECOVERY_STAND)
    {
        if (trigger)
        {
            printf("MODE TRIGGER!\n");
        }
        rc_control.mode = selected_mode;
    }
}

void sbus_packet_complete_logitech()
{
    logitech_data data;
    update_logitech_data(&data);

    if (data.LB > BUTTERN_PRESS_THRESHOULD && data.START > BUTTERN_PRESS_THRESHOULD)
        selected_mode_logitech = RC_mode::PASSIVE;
    else if (data.LB > BUTTERN_PRESS_THRESHOULD && data.BACK > BUTTERN_PRESS_THRESHOULD)
        selected_mode_logitech = RC_mode::OFF;
    else if (data.LB > BUTTERN_PRESS_THRESHOULD && data.RB > BUTTERN_PRESS_THRESHOULD)
        selected_mode_logitech = RC_mode::LOCK_JOINT;
    else if (data.LB > BUTTERN_PRESS_THRESHOULD && data.A > BUTTERN_PRESS_THRESHOULD)
        selected_mode_logitech = RC_mode::STAND_UP;
    else if (data.LB > BUTTERN_PRESS_THRESHOULD && data.B > BUTTERN_PRESS_THRESHOULD)
        selected_mode_logitech = RC_mode::BALANCE_STAND;
    else if (data.LB > BUTTERN_PRESS_THRESHOULD && data.X > BUTTERN_PRESS_THRESHOULD && rc_control.mode == RC_mode::STAND_UP)
        selected_mode_logitech = RC_mode::LOCOMOTION;
    else // disable the other buttons, if the other buttons are triggered, the current mode will be kept.
    {
        selected_mode_logitech = rc_control.mode;
    }

    if (selected_mode_logitech == RC_mode::LOCOMOTION)
    {
        rc_control.bias_save_mode_pre = rc_control.bias_save_mode;
        rc_control.v_des[0] = data.leftStickXAnalog;
        rc_control.v_des[1] = data.leftStickYAnalog;
        rc_control.v_des[2] = 0;

        rc_control.omega_des[0] = 0;
        rc_control.omega_des[1] = 0;
        rc_control.omega_des[2] = data.rightStickYAnalog;
        if (data.A > BUTTERN_PRESS_THRESHOULD && data.A > rc_control.last_gait_type)
        {
            rc_control.gait_type = 1 - rc_control.gait_type;
        }
        rc_control.last_gait_type = data.A;

        if (data.B > BUTTERN_PRESS_THRESHOULD && data.B > rc_control.last_imu_euler_angle_calibration_mode && rc_control.imu_linvel_calibration_mode == 0)
        {
            rc_control.imu_euler_angle_calibration_mode = 1 - rc_control.imu_euler_angle_calibration_mode;

            switch (rc_control.imu_euler_angle_calibration_mode)
            {
            case 0:
                cout << "[--Turn Off euler_angle_bias_calib_mode--]" << endl;
                rc_control.bias_save_mode = false;
                break;
            case 1:
                cout << "[--Turn On euler_angle_bias_calib_mode--]" << endl;
                break;
            default:
                break;
            }
        }

        if (rc_control.imu_euler_angle_calibration_mode > 0)
        {
            // std::cout << "----------------------------------------" << std::endl;
            // std::cout << "[IMU EulerAngle Bias Calibration Mode]" << std::endl;

            int imu_flag = 0;
            if (data.CROSS_X > BUTTERN_PRESS_THRESHOULD && data.CROSS_X > rc_control.last_crossx)
            {
                rc_control.pitch_bias -= 0.001;
                imu_flag += 1;
            }
            if (data.CROSS_X < -BUTTERN_PRESS_THRESHOULD && data.CROSS_X < rc_control.last_crossx)
            {
                rc_control.pitch_bias += 0.001;
                imu_flag += 1;
            }
            if (data.CROSS_Y > BUTTERN_PRESS_THRESHOULD && data.CROSS_Y > rc_control.last_crossy)
            {
                rc_control.roll_bias += 0.001;
                imu_flag += 1;
            }
            if (data.CROSS_Y < -BUTTERN_PRESS_THRESHOULD && data.CROSS_Y < rc_control.last_crossy)
            {
                rc_control.roll_bias -= 0.001;
                imu_flag += 1;
            }

            if (imu_flag > 0)
            {
                std::cout << "[Pitch Bias]: " << rc_control.pitch_bias << " [Roll Bias]:" << rc_control.roll_bias << std::endl;
                std::cout << "[Bias Save Mode]: " << rc_control.bias_save_mode << std::endl;
            }
        }
        rc_control.last_imu_euler_angle_calibration_mode = data.B;
        rc_control.last_crossx = data.CROSS_X;
        rc_control.last_crossy = data.CROSS_Y;

        if (data.RB > BUTTERN_PRESS_THRESHOULD && data.RB > rc_control.last_imu_linvel_calibration_mode && rc_control.imu_euler_angle_calibration_mode == 0)
        {
            rc_control.imu_linvel_calibration_mode = 1 - rc_control.imu_linvel_calibration_mode;

            switch (rc_control.imu_linvel_calibration_mode)
            {
            case 0:
                cout << "[--Turn Off linvel_bias_calib_mode--]" << endl;
                rc_control.bias_save_mode = false;
                break;
            case 1:
                cout << "[--Turn On linvel_bias_calib_mode--]" << endl;
                break;
            default:
                break;
            }
        }
        if (rc_control.imu_linvel_calibration_mode > 0)
        {
            // std::cout << "----------------------------------------" << std::endl;
            // std::cout << "[IMU Linear Velocity Bias Calibration Mode]" << rc_control.imu_linvel_calibration_mode << std::endl;
            int vel_flag = 0;
            if (data.CROSS_X > BUTTERN_PRESS_THRESHOULD && data.CROSS_X > rc_control.last_vel_crossx)
            {
                rc_control.cmd_vel_bias_x += 0.001;
                vel_flag += 1;
            }
            if (data.CROSS_X < -BUTTERN_PRESS_THRESHOULD && data.CROSS_X < rc_control.last_vel_crossx)
            {
                rc_control.cmd_vel_bias_x -= 0.001;
                vel_flag += 1;
            }
            if (data.CROSS_Y > BUTTERN_PRESS_THRESHOULD && data.CROSS_Y > rc_control.last_vel_crossy)
            {
                rc_control.cmd_vel_bias_y += 0.001;
                vel_flag += 1;
            }
            if (data.CROSS_Y < -BUTTERN_PRESS_THRESHOULD && data.CROSS_Y < rc_control.last_vel_crossy)
            {
                rc_control.cmd_vel_bias_y -= 0.001;
                vel_flag += 1;
            }

            if (vel_flag > 0)
            {
                std::cout << "[VelX Bias]: " << rc_control.cmd_vel_bias_x << " [VelY Bias]:" << rc_control.cmd_vel_bias_y << std::endl;
                std::cout << "[Bias Save Mode]: " << rc_control.bias_save_mode << std::endl;
            }
        }
        rc_control.last_imu_linvel_calibration_mode = data.RB;
        rc_control.last_vel_crossx = data.CROSS_X;
        rc_control.last_vel_crossy = data.CROSS_Y;

        if (data.Y > BUTTERN_PRESS_THRESHOULD && !rc_control.bias_save_mode) // LB + X enable the bias save mode
        {
            rc_control.bias_save_mode = true;
        }
        else
        {
            rc_control.bias_save_mode = false;
        }

        if (data.X > BUTTERN_PRESS_THRESHOULD && !rc_control.bias_clear_mode)
        {
            if (rc_control.imu_euler_angle_calibration_mode)
            {
                rc_control.roll_bias = 0;
                rc_control.pitch_bias = 0;
            }
            if (rc_control.imu_linvel_calibration_mode)
            {
                rc_control.cmd_vel_bias_x = 0;
                rc_control.cmd_vel_bias_y = 0;
            }

            rc_control.bias_clear_mode = true;
        }
        else
        {
            rc_control.bias_clear_mode = false;
        }
    }

    // std::cout << "--------------------" << std::endl;
    // cout << "[rt_rc_interface] vX: " << rc_control.v_des[0] << endl;
    // cout << "[rt_rc_interface] vY: " << rc_control.v_des[1] << endl;
    // cout << "[rt_rc_interface] omegaZ: " << rc_control.omega_des[2] << endl;
    // cout << "*********** selected_mode ***********" << endl;
    // cout << "data.START: " << data.START << endl;
    // cout << "data.LB: " << data.LB << endl;
    // cout << "data.A: " << data.A << endl;
    // cout << "data.X: " << data.X << endl;
    // cout << "data.Y: " << data.Y << endl;

    // switch (selected_mode_logitech)
    // {
    // case 0:
    //     cout << "RC_mode::OFF" << endl;
    //     break;
    // case 1:
    //     cout << "RC_mode::PASSIVE" << endl;
    //     break;
    // case 2:
    //     cout << "RC_mode::STAND_UP " << endl;
    //     break;
    // case 11:
    //     cout << "RC_mode::LOCOMOTION" << endl;
    //     break;
    // default:
    //     break;
    // }
    // cout << endl;
    // cout << "*********** rc_control_mode ***********" << endl;
    // switch (int(rc_control.mode))
    // {
    // case 0:
    //     cout << "RC_mode::OFF" << endl;
    //     break;
    // case 1:
    //     cout << "RC_mode::PASSIVE" << endl;
    //     break;
    // case 2:
    //     cout << "RC_mode::STAND_UP " << endl;
    //     break;
    // case 11:
    //     cout << "RC_mode::LOCOMOTION" << endl;
    //     break;
    // default:
    //     break;
    // }

    rc_control.mode = selected_mode_logitech;
}

void *v_memcpy(void *dest, volatile void *src, size_t n)
{
    void *src_2 = (void *)src;
    return memcpy(dest, src_2, n);
}

float deadband(float command, float deadbandRegion, float minVal, float maxVal)
{
    if (command < deadbandRegion && command > -deadbandRegion)
    {
        return 0.0;
    }
    else
    {
        return (command / (2)) * (maxVal - minVal);
    }
}
