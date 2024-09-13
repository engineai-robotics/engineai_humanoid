//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_CONTROLFSMDATA_H
#define ZQ_HUMANOID_CONTROLFSMDATA_H

#include "common/include/ControlParameters/RobotParameters.h"
#include "../EngineAI_UserParameters.h"
#include "common/include/Controllers/DesiredStateCommand.h"
#include "common/include/Controllers/GaitScheduler.h"
#include "common/include/Controllers/LegController.h"
#include "common/include/Controllers/StateEstimatorContainer.h"
#include "common/include/Dynamics/RobotConstructor.h"

/**
 *
 */
template <typename T>
struct ControlFSMData
{
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RobotConstructor<T> *_humanoid_biped;
    StateEstimatorContainer<T> *_stateEstimator;
    LegController<T> *_legController;
    GaitScheduler<T> *_gaitScheduler;
    DesiredStateCommand<T> *_desiredStateCommand;
    RobotControlParameters *controlParameters;
    EngineAI_UserParameters *userParameters;
    VisualizationData *visualizationData;
};

template struct ControlFSMData<double>;
template struct ControlFSMData<float>;

#endif // ZQ_HUMANOID_CONTROLFSMDATA_H
