//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_CONTROLPARAMETERINTERFACE_H
#define ZQ_HUMANOID_CONTROLPARAMETERINTERFACE_H

#include <map>
#include "ControlParameters.h"

/*!
 * Type of message to a control parameter collection
 */
enum class ControlParameterRequestKind
{
    GET_ROBOT_PARAM_BY_NAME,
    SET_ROBOT_PARAM_BY_NAME,
    GET_USER_PARAM_BY_NAME,
    SET_USER_PARAM_BY_NAME
};

std::string controlParameterRequestKindToString(
    ControlParameterRequestKind request);

/*!
 * Data sent to a control parameter collection to request a get/set of a value
 */
struct ControlParameterRequest
{
    char name[CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH] =
        ""; // name of the parameter to set/get
    u64 requestNumber = UINT64_MAX;
    ControlParameterValue value;
    ControlParameterValueKind parameterKind;
    ControlParameterRequestKind requestKind;

    /*!
     * Convert to human-readable string
     * @return : description of the request
     */
    std::string toString()
    {
        std::string result = "Request(" + std::to_string(requestNumber) + ") " +
                             controlParameterRequestKindToString(requestKind) +
                             " " +
                             controlParameterValueKindToString(parameterKind) +
                             " " + std::string(name) + " ";
        switch (requestKind)
        {
        case ControlParameterRequestKind::GET_USER_PARAM_BY_NAME:
            result += "user is: ";
            result += controlParameterValueToString(value, parameterKind);
            return result;
        case ControlParameterRequestKind::SET_USER_PARAM_BY_NAME:
            result += "user to: ";
            result += controlParameterValueToString(value, parameterKind);
            return result;
        case ControlParameterRequestKind::GET_ROBOT_PARAM_BY_NAME:
            result += "robot is: ";
            result += controlParameterValueToString(value, parameterKind);
            return result;
        case ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME:
            result += "robot to: ";
            result += controlParameterValueToString(value, parameterKind);
            return result;
        default:
            return result + " unknown request type!";
        }
    }
};

/*!
 * Data sent from a control parameter collection in response to a request.
 */
struct ControlParameterResponse
{
    char name[CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH] = "";
    u64 requestNumber = UINT64_MAX;
    u64 nParameters = 0;
    ControlParameterValue value;
    ControlParameterValueKind parameterKind;
    ControlParameterRequestKind requestKind;

    /*!
     * Check if a response is a valid response to a given request
     * @param request : the request
     * @return is the response from the given request?
     */
    bool isResponseTo(ControlParameterRequest &request)
    {
        return requestNumber == request.requestNumber &&
               requestKind == request.requestKind &&
               std::string(name) == std::string(request.name);
    }

    /*!
     * Convert to human-readable string
     * @return : description of the response
     */
    std::string toString()
    {
        std::string result = "Response(" + std::to_string(requestNumber) + ") " +
                             controlParameterRequestKindToString(requestKind) +
                             " " +
                             controlParameterValueKindToString(parameterKind) +
                             " " + std::string(name) + " ";

        switch (requestKind)
        {
        case ControlParameterRequestKind::SET_USER_PARAM_BY_NAME:
            result += "user to: ";
            result += controlParameterValueToString(value, parameterKind);
            return result;
        case ControlParameterRequestKind::GET_USER_PARAM_BY_NAME:
            result += "user is: ";
            result += controlParameterValueToString(value, parameterKind);
            return result;
        case ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME:
            result += "robot to: ";
            result += controlParameterValueToString(value, parameterKind);
            return result;
        case ControlParameterRequestKind::GET_ROBOT_PARAM_BY_NAME:
            result += "robot is: ";
            result += controlParameterValueToString(value, parameterKind);
            return result;
        default:
            return result + " unknown request type!";
        }
    }
};
#endif // ZQ_HUMANOID_CONTROLPARAMETERINTERFACE_H
