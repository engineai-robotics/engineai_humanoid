/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <cassert>
#include <iostream>
#include "HardwareBridge.h"
#include "SimulationBridge.h"
#include "main_helper.h"
#include "RobotController.h"
#define USE_REAL_ROBOT
MasterConfig gMasterConfig;

/*!
 * Setup and run the given robot controller
 */

SimulationBridge *simulationBridge; //(gMasterConfig._robot, ctrl);

int main_helper(RobotController *ctrl)
{
#ifndef USE_REAL_ROBOT
  static int flag_ini = 0;
  if (flag_ini == 0)
  {
    //        gMasterConfig._robot=RobotType::ZQ_Biped_SA01;
    gMasterConfig._robot = RobotType::ZQ_Biped_SA01P;
    simulationBridge = new SimulationBridge(gMasterConfig._robot, ctrl);
    flag_ini = 1;
  }
  simulationBridge->run();
#else
  printf("main_helper begin run ZqSA01HardwareBridge\n");
  ZqSA01HardwareBridge hw(ctrl, gMasterConfig.load_from_file);
  printf("hw.run() begin run \n");
  hw.run();
#endif

  return 0;
}
