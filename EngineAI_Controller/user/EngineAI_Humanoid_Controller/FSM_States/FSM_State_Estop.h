//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_FSM_STATE_ESTOP_H
#define ZQ_HUMANOID_FSM_STATE_ESTOP_H

#include "FSM_State.h"
#include "motor.h"

/**
 *
 */
template <typename T>
class FSM_State_Estop : public FSM_State<T>
{
public:
    FSM_State_Estop(ControlFSMData<T> *_controlFSMData);

    // Behavior to be carried out when entering a state
    void onEnter();

    // Run the normal behavior for the state
    void run();

    // Checks for any transition triggers
    FSM_StateName checkTransition();

    // Manages state specific transitions
    TransitionData<T> transition();

    // Behavior to be carried out when exiting a state
    void onExit();

    TransitionData<T> testTransition();

private:
    // Keep track of the control iterations
    int iter = 0;
};

#endif // ZQ_HUMANOID_FSM_STATE_ESTOP_H
