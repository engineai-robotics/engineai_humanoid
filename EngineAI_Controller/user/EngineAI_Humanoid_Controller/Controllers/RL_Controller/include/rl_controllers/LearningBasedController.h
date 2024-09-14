#pragma once

#include <iostream>
#include "Types.h"
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <atomic>
#include <map>
#include <iostream>
#include "../../../../FSM_States/ControlFSMData.h"
#include <yaml-cpp/yaml.h>
#include "Lowpassfilter.h"
#include "common/include/Controllers/close_chain_mapping.h"

class LearningBasedController
{
public:
  LearningBasedController(ControlFSMData<float> *control_FSM_data) : _memoryInfo(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
  {
    _control_FSM_data = control_FSM_data;
    // Load policy model and rl config
    if (!loadPolicyModel())
    {
      std::cout << "policy load failed!!" << std::endl;
    }

    if (!loadRLConfig())
    {
      std::cout << "RL config file load failed!!" << std::endl;
    }

    _ankle_joint_mapping = new Decouple();
  }

  ~LearningBasedController();

  bool init();
  void update();

  bool loadPolicyModel();
  bool loadRLConfig();
  void updateRobotState();
  void policyInference();
  void computeActions();
  void updateBias();
  void computeObservation();
  void updateMotorCommand();

  void printRobotConfig();

private:
  // onnx policy model
  std::string _policyFileDirectory;

  std::shared_ptr<Ort::Env> _onnxEnvPtr;
  std::unique_ptr<Ort::Session> _sessionPtr;

  std::vector<const char *> _inputNames;
  std::vector<const char *> _outputNames;

  std::vector<Ort::AllocatedStringPtr> _inputNodeNameAllocatedStrings;
  std::vector<Ort::AllocatedStringPtr> _outputNodeNameAllocatedStrings;

  std::vector<std::vector<int64_t>> _inputShapes;
  std::vector<std::vector<int64_t>> _outputShapes;

  VecDynamic _lastActions;
  VecDynamic _defaultJointPos;
  Decouple *_ankle_joint_mapping = nullptr;

  bool _isfirstRecObs{true};
  int _actionsSize;
  int _observationSizeSingleFrame;
  int _observationSizeFrameStack;

  std::vector<float> _actions;
  std::vector<float> _observations;
  Ort::MemoryInfo _memoryInfo;
  Eigen::Matrix<float, Eigen::Dynamic, 1> _observationHistoryBuffer;
  bool _isfirstCompAct{true};

  int64_t _loopCount;
  Command _command;
  LearningBasedRobotConfig _robotConfig{};

  StateCollector _stateCollection;

  int actuatedDofNum_ = 12;

  double gait_phase_;

  bool still_flag = false;

  size_t _joint_dim{0};

  ControlFSMData<float> *_control_FSM_data;

  JointDatas _joint_datas;

  YAML::Node _config_params_loader;
  float cmd_bias_vel_x = 0;
  float cmd_bias_vel_y = 0;
  float euler_roll_bias = 0;
  float euler_pitch_bias = 0;
  bool save_yaml_config;
  bool bias_clear_flag;

  /*tau mapping test*/
  Eigen::VectorXd q = Eigen::MatrixXd::Zero(12, 1);
  Eigen::VectorXd vel = Eigen::MatrixXd::Zero(12, 1);
  Eigen::VectorXd tau = Eigen::MatrixXd::Zero(12, 1);

  //
  std::vector<std::string> jointNames{"leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
                                      "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint", "leg_r6_joint"};

  // close chain params
  double _joint_pos_state[12] = {0};
  double _motor_pos_state[12] = {0};

  double _joint_pos_cmd[12] = {0};
  double _joint_vel_cmd[12] = {0};
  double _joint_tauff_cmd[12] = {0};
  std::unique_ptr<LowPassFilter<Eigen::Matrix<float, 12, 1>>> _lpf_action;
  Eigen::Matrix<float, 12, 1> _lpf_mlp_net_action_;
};
