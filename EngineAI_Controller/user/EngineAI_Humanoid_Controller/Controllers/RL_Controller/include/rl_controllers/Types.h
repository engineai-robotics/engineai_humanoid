#pragma once

#include <map>
#include <atomic>
#include <eigen3/Eigen/Dense>

using VecDynamic = Eigen::Matrix<float, Eigen::Dynamic, 1>;
using MatDynamic = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using Vec3_RL = Eigen::Matrix<float, 3, 1>;
using Mat3_RL = Eigen::Matrix<float, 3, 3>;
using Quat_RL = Eigen::Quaternion<float>;

struct LearningBasedRobotConfig
{
    struct ControlConfig
    {
        std::map<std::string, float> stiffness;
        std::map<std::string, float> damping;
        float actionScale;
        int decimation;
        float gait_period;
    };

    struct DefaultJointPos
    {
        // default joint angles
        float leg_l1_joint;
        float leg_l2_joint;
        float leg_l3_joint;
        float leg_l4_joint;
        float leg_l5_joint;
        float leg_l6_joint;
        float leg_r1_joint;
        float leg_r2_joint;
        float leg_r3_joint;
        float leg_r4_joint;
        float leg_r5_joint;
        float leg_r6_joint;
    };

    struct ObsScales
    {
        float linVel;
        float angVel;
        float dofPos;
        float dofVel;
        float quat;
    };

    bool enable_filter;
    float still_ratio;
    float clipActions;
    float clipObs;
    bool enable_ankle_tau_mapping;

    DefaultJointPos defaultJointPos;
    ObsScales obsScales;
    ControlConfig controlConfig;

    float control_frequency;
    float joy_linvX_scale;
    float joy_linvY_scale;
    float joy_omegaZ_scale;
    float velx_bias;
    float vely_bias;
    float roll_bias;
    float pitch_bias;
};

struct StateCollector
{
    VecDynamic jointPos;
    VecDynamic jointVel;
    Vec3_RL torsoAngVel;
    Vec3_RL torsoEulerXyz;
    Vec3_RL projectedGravity;
};

struct Command
{
    std::atomic<float> x;
    std::atomic<float> y;
    std::atomic<float> yaw;
};

struct JointDatas
{
    Eigen::VectorXd q = Eigen::MatrixXd::Zero(12, 1);
    Eigen::VectorXd qd = Eigen::MatrixXd::Zero(12, 1);
    Eigen::VectorXd tau_state = Eigen::MatrixXd::Zero(12, 1);

    Eigen::VectorXd q_des = Eigen::MatrixXd::Zero(12, 1);
    Eigen::VectorXd qd_des = Eigen::MatrixXd::Zero(12, 1);
    Eigen::VectorXd tau_ff = Eigen::MatrixXd::Zero(12, 1);
    Eigen::VectorXd Kp = Eigen::MatrixXd::Zero(12, 1);
    Eigen::VectorXd Kd = Eigen::MatrixXd::Zero(12, 1);
};