//
// Created by engineai on 2024/07/03.
//
/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 */

#include "SafetyChecker.h"

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkSafeOrientation()
{ // roll: limited 50degree ; pitch limited  70degree
    return true;

    if (fabs(data->_stateEstimator->getResult().rpy(0)) >= 0.87 ||
        fabs(data->_stateEstimator->getResult().rpy(1)) >= 1.22)
    {
        printf("Orientation safety check failed! roll : %.2f\t  pitch :%.2f\n", data->_stateEstimator->getResult().rpy(0),
               data->_stateEstimator->getResult().rpy(1));
        return false;
    }
    else
    {
        return true;
    }
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkPDesFoot()
{
    // Assumed safe to start
    bool safePDesFoot = true;

    // Safety parameters
    T maxAngle = 1.0472; // 60 degrees (should be changed)
    T maxPDes = data->_humanoid_biped->_maxLegLength * sin(maxAngle);

    // Check all of the legs
    for (int leg = 0; leg < 4; leg++)
    {
        // Keep the foot from going too far from the body in +x
        if (data->_legController->commands[leg].pDes(0) > maxPDes)
        {
            std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                      << " | coordinate: " << 0 << "\n";
            std::cout << "   commanded: "
                      << data->_legController->commands[leg].pDes(0)
                      << " | modified: " << maxPDes << std::endl;
            data->_legController->commands[leg].pDes(0) = maxPDes;
            safePDesFoot = false;
        }

        // Keep the foot from going too far from the body in -x
        if (data->_legController->commands[leg].pDes(0) < -maxPDes)
        {
            std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                      << " | coordinate: " << 0 << "\n";
            std::cout << "   commanded: "
                      << data->_legController->commands[leg].pDes(0)
                      << " | modified: " << -maxPDes << std::endl;
            data->_legController->commands[leg].pDes(0) = -maxPDes;
            safePDesFoot = false;
        }

        // Keep the foot from going too far from the body in +y
        if (data->_legController->commands[leg].pDes(1) > maxPDes)
        {
            std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                      << " | coordinate: " << 1 << "\n";
            std::cout << "   commanded: "
                      << data->_legController->commands[leg].pDes(1)
                      << " | modified: " << maxPDes << std::endl;
            data->_legController->commands[leg].pDes(1) = maxPDes;
            safePDesFoot = false;
        }

        // Keep the foot from going too far from the body in -y
        if (data->_legController->commands[leg].pDes(1) < -maxPDes)
        {
            std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                      << " | coordinate: " << 1 << "\n";
            std::cout << "   commanded: "
                      << data->_legController->commands[leg].pDes(1)
                      << " | modified: " << -maxPDes << std::endl;
            data->_legController->commands[leg].pDes(1) = -maxPDes;
            safePDesFoot = false;
        }

        // Keep the leg under the motor module (don't raise above body or crash into
        // module)
        if (data->_legController->commands[leg].pDes(2) >
            -data->_humanoid_biped->_maxLegLength / 4)
        {
            std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                      << " | coordinate: " << 2 << "\n";
            std::cout << "   commanded: "
                      << data->_legController->commands[leg].pDes(2)
                      << " | modified: " << -data->_humanoid_biped->_maxLegLength / 4
                      << std::endl;
            data->_legController->commands[leg].pDes(2) =
                -data->_humanoid_biped->_maxLegLength / 4;
            safePDesFoot = false;
        }

        // Keep the foot within the kinematic limits
        if (data->_legController->commands[leg].pDes(2) <
            -data->_humanoid_biped->_maxLegLength)
        {
            std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                      << " | coordinate: " << 2 << "\n";
            std::cout << "   commanded: "
                      << data->_legController->commands[leg].pDes(2)
                      << " | modified: " << -data->_humanoid_biped->_maxLegLength
                      << std::endl;
            data->_legController->commands[leg].pDes(2) =
                -data->_humanoid_biped->_maxLegLength;
            safePDesFoot = false;
        }
    }

    // Return true if all desired positions are safe
    return safePDesFoot;
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkForceFeedForward()
{
    // Assumed safe to start
    bool safeForceFeedForward = true;

    // Initialize maximum vertical and lateral forces
    T maxLateralForce = 0;
    T maxVerticalForce = 0;

    // Maximum force limits for each robot
    if (data->_humanoid_biped->_robotType == RobotType::ZQ_Biped_SA01)
    {
        maxLateralForce = 1800;
        maxVerticalForce = 1800;
    }
    else if (data->_humanoid_biped->_robotType == RobotType::ZQ_Biped_SA01P)
    {
        maxLateralForce = 1000;
        maxVerticalForce = 1000;
    }

    // Check all of the legs
    for (int leg = 0; leg < 4; leg++)
    {
        // Limit the lateral forces in +x body frame
        if (data->_legController->commands[leg].forceFeedForward(0) > maxLateralForce)
        {
#ifdef USE_LOG_DEBUG_HL
            std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                      << " | coordinate: " << 0 << "\n";
            std::cout << "   commanded: "
                      << data->_legController->commands[leg].forceFeedForward(0)
                      << " | modified: " << maxLateralForce << std::endl;
#endif
            data->_legController->commands[leg].forceFeedForward(0) = maxLateralForce;
            safeForceFeedForward = false;
        }

        // Limit the lateral forces in -x body frame
        if (data->_legController->commands[leg].forceFeedForward(0) <
            -maxLateralForce)
        {
#ifdef USE_LOG_DEBUG_HL
            std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                      << " | coordinate: " << 0 << "\n";
            std::cout << "   commanded: "
                      << data->_legController->commands[leg].forceFeedForward(0)
                      << " | modified: " << -maxLateralForce << std::endl;
#endif
            data->_legController->commands[leg].forceFeedForward(0) =
                -maxLateralForce;
            safeForceFeedForward = false;
        }

        // Limit the lateral forces in +y body frame
        if (data->_legController->commands[leg].forceFeedForward(1) >
            maxLateralForce)
        {
#ifdef USE_LOG_DEBUG_HL
            std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                      << " | coordinate: " << 1 << "\n";
            std::cout << "   commanded: "
                      << data->_legController->commands[leg].forceFeedForward(1)
                      << " | modified: " << maxLateralForce << std::endl;
#endif
            data->_legController->commands[leg].forceFeedForward(1) = maxLateralForce;
            safeForceFeedForward = false;
        }

        // Limit the lateral forces in -y body frame
        if (data->_legController->commands[leg].forceFeedForward(1) <
            -maxLateralForce)
        {
#ifdef USE_LOG_DEBUG_HL
            std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                      << " | coordinate: " << 1 << "\n";
            std::cout << "   commanded: "
                      << data->_legController->commands[leg].forceFeedForward(1)
                      << " | modified: " << -maxLateralForce << std::endl;
#endif
            data->_legController->commands[leg].forceFeedForward(1) =
                -maxLateralForce;
            safeForceFeedForward = false;
        }

        // Limit the vertical forces in +z body frame
        if (data->_legController->commands[leg].forceFeedForward(2) > maxVerticalForce)
        {
#ifdef USE_LOG_DEBUG_HL
            std::cout << "[CONTROL FSM] Safety: Force leg: " << leg << " | coordinate: " << 2 << "\n";
            std::cout << "   commanded: " << data->_legController->commands[leg].forceFeedForward(2)
                      << " | modified: " << -maxVerticalForce << std::endl;
#endif
            data->_legController->commands[leg].forceFeedForward(2) =
                maxVerticalForce;
            safeForceFeedForward = false;
        }

        // Limit the vertical forces in -z body frame
        if (data->_legController->commands[leg].forceFeedForward(2) <
            -maxVerticalForce)
        {
#ifdef USE_LOG_DEBUG_HL
            std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                      << " | coordinate: " << 2 << "\n";
            std::cout << "   commanded: "
                      << data->_legController->commands[leg].forceFeedForward(2)
                      << " | modified: " << maxVerticalForce << std::endl;
#endif
            data->_legController->commands[leg].forceFeedForward(2) =
                -maxVerticalForce;
            safeForceFeedForward = false;
        }
    }

    // Return true if all feed forward forces are safe
    return safeForceFeedForward;
}

// template class SafetyChecker<double>; This should be fixed... need to make
// RobotRunner a template
template class SafetyChecker<float>;
