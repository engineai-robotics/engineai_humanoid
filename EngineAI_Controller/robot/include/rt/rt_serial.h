//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_RT_SERIAL_H
#define ZQ_HUMANOID_RT_SERIAL_H
int set_interface_attribs_custom_baud(int fd, int speed, int parity, int port);
void init_serial_for_sbus(int fd, int baud);
#endif // ZQ_HUMANOID_RT_SERIAL_H
