/*!
 * @file main_helper.h
 * @brief Function which should be called in main to start your robot control code
 */

#ifndef ROBOT_MAIN_H
#define ROBOT_MAIN_H

#include "common/include/Types.h"
#include "RobotController.h"
//#include "SimulationBridge.h"
extern MasterConfig gMasterConfig;
int main_helper(RobotController* ctrl);
//SimulationBridge*  main_helper(RobotController* ctrl);

#endif  // ROBOT_MAIN_H
