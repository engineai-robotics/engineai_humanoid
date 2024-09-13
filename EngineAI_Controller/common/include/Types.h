//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_TYPES_H
#define ZQ_HUMANOID_TYPES_H

#include "cppTypes.h"
struct MasterConfig
{
    RobotType _robot;
    bool simulated = false;
    bool load_from_file = false;
};
#endif // ZQ_HUMANOID_TYPES_H
