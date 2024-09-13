//
// Created by engineai on 2024/07/03.
//
/*!
 * @file SegfaultHandler.h
 * @brief Handler for segfaults.
 * This will catch a segfault, print a stack trace, and put an error code in the shared memory
 * (if it is connected), so that the simulator can provide a reasonable error message for why the
 * robot code disappears.
 */
#ifndef ZQ_HUMANOID_SEGFAULTHANDLER_H
#define ZQ_HUMANOID_SEGFAULTHANDLER_H

#include <cstdint>

void install_segfault_handler(char *error_message);

#endif // ZQ_HUMANOID_SEGFAULTHANDLER_H
