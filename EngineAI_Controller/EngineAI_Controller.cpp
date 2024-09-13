#include <time.h>
#include <math.h>
#include "robot/include/main_helper.h"
#include "user/EngineAI_Humanoid_Controller/EngineAI_Humanoid_Controller.h"
#include "robot/include/RobotController.h"
#include <time.h>
#include <unistd.h>

int main(int argc, char **argv)
{
  printf("ZQ_Robot_Controller main() begin ... \n");

  RobotController *robot_controller = new EngineAI_Humanoid_Controller();

  main_helper(robot_controller);

  // delete robot;
  return 0;
}
