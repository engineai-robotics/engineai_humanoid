//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_RT_SBUS_H
#define ZQ_HUMANOID_RT_SBUS_H
#include <stdint.h>
#include "../../../third-party/LogitechGamepad/src/Logitech_controller.h"

#define RC_AT9s
#define K_USE_LOGITECH_GAMEPAD

void unpack_sbus_data(uint8_t sbus_data[], uint16_t *channels);

int read_sbus_data(int port, uint8_t *sbus_data);

int read_sbus_channel(int channel);

int receive_sbus(int port);

int init_sbus(int is_simulator);

enum TaranisSwitchState
{
    SWITCH_UP = 0,
    SWITCH_MIDDLE = 1,
    SWITCH_DOWN = 2,
};
enum AT9s_SwitchStateBool
{
    AT9S_BOOL_UP = 0,
    AT9S_BOOL_DOWN = 1,
};
enum AT9s_SwitchStateTri
{
    AT9S_TRI_UP = 0,
    AT9S_TRI_MIDDLE = 1,
    AT9S_TRI_DOWN = 2,
};

struct Taranis_X7_data
{
    TaranisSwitchState left_upper_switch, left_lower_left_switch, left_lower_right_switch,
        right_upper_switch, right_lower_left_switch, right_lower_right_switch;

    float left_stick[2];
    float right_stick[2];
    float knobs[2];
};

struct AT9s_data
{
    AT9s_SwitchStateBool SWF, SWA, SWB, SWD;
    AT9s_SwitchStateTri SWE, SWC, SWG;
    float left_stick_x, left_stick_y;
    float right_stick_x, right_stick_y;
    float varB;
};

void update_taranis_x7(Taranis_X7_data *data);

void update_taranis_at9s(AT9s_data *data);

void update_logitech_data(logitech_data *data);

#endif // ZQ_HUMANOID_RT_SBUS_H
