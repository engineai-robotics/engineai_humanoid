#include "../include/rl_controllers/LearningBasedController.h"
#include "../include/rl_controllers/RotationTools.h"
#include <algorithm>
#include <string.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

LearningBasedController::~LearningBasedController()
{
}
bool LearningBasedController::init()
{
  actuatedDofNum_ = jointNames.size();

  _loopCount = 0;

  _joint_dim = actuatedDofNum_;

  _isfirstCompAct = true;
  _lpf_mlp_net_action_.setZero();

  save_yaml_config = true;

  printRobotConfig();

  return true;
}

void LearningBasedController::update()
{
  updateRobotState(); // update robot state

  updateBias(); // update cmd and euler angle bias

  policyInference(); // inference for action

  updateMotorCommand(); // update the robot joint command

  _loopCount++;
}

void LearningBasedController::policyInference()
{
  // std::cout << "get into LearningBasedController::policyInference" << std::endl;
  //  compute observation & actions
  if (std::cout.fail())
  {
    std::cerr << "std::cout is in a bad state!" << std::endl;
    std::cout.clear();
  }

  if (_loopCount % _robotConfig.controlConfig.decimation == 0)
  {
    computeObservation();
    computeActions();
    // limit action range
    float actionMin = -_robotConfig.clipActions;
    float actionMax = _robotConfig.clipActions;
    std::transform(_actions.begin(), _actions.end(), _actions.begin(),
                   [actionMin, actionMax](float x)
                   { return std::max(actionMin, std::min(actionMax, x)); });
  }

  // set action
  for (int i = 0; i < _actionsSize; i++)
  {
    std::string jointName = jointNames[i];

    float pos_des = _actions[i] * _robotConfig.controlConfig.actionScale + _defaultJointPos(i);
    double stiffness = _robotConfig.controlConfig.stiffness[jointName];
    double damping = _robotConfig.controlConfig.damping[jointName];

    _joint_datas.q_des[i] = pos_des;
    _joint_datas.qd_des[i] = 0.0;
    _joint_datas.tau_ff[i] = 0.0;
    _joint_datas.Kp[i] = stiffness;
    _joint_datas.Kd[i] = damping;

    _lastActions(i, 0) = _actions[i];
  }
}

bool LearningBasedController::loadPolicyModel()
{
  std::string policyFilePath = "../policy/zqsa01/zqsa01_policy.onnx";

  _policyFileDirectory = policyFilePath;
  std::cout << "Load Onnx model from path : " << policyFilePath << std::endl;

  // create env
  _onnxEnvPtr.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
  // create session
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetIntraOpNumThreads(1);
  _sessionPtr = std::make_unique<Ort::Session>(*_onnxEnvPtr, policyFilePath.c_str(), sessionOptions);
  // get input and output info
  _inputNames.clear();
  _outputNames.clear();
  _inputShapes.clear();
  _outputShapes.clear();
  Ort::AllocatorWithDefaultOptions allocator;

  for (size_t i = 0; i < _sessionPtr->GetInputCount(); i++)
  {
    auto inputnamePtr = _sessionPtr->GetInputNameAllocated(i, allocator);
    _inputNodeNameAllocatedStrings.push_back(std::move(inputnamePtr));
    _inputNames.push_back(_inputNodeNameAllocatedStrings.back().get());

    _inputShapes.push_back(_sessionPtr->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    std::vector<int64_t> shape = _sessionPtr->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
  }
  for (size_t i = 0; i < _sessionPtr->GetOutputCount(); i++)
  {
    auto outputnamePtr = _sessionPtr->GetOutputNameAllocated(i, allocator);
    _outputNodeNameAllocatedStrings.push_back(std::move(outputnamePtr));
    _outputNames.push_back(_outputNodeNameAllocatedStrings.back().get());
    std::cout << _sessionPtr->GetOutputNameAllocated(i, allocator).get() << std::endl;
    _outputShapes.push_back(_sessionPtr->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    std::vector<int64_t> shape = _sessionPtr->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
  }

  std::cout << "Load Onnx model successfully !!!" << std::endl;
  return true;
}

bool LearningBasedController::loadRLConfig()
{

  // load config file
  _config_params_loader = YAML::LoadFile("../config/zqsa01_rl.yaml");

  // control frequency
  _robotConfig.control_frequency = _config_params_loader["LeggedRobotRLConfig"]["control_frequency"].as<float>();
  _robotConfig.enable_filter = _config_params_loader["LeggedRobotRLConfig"]["enable_filter"].as<bool>();
  _robotConfig.still_ratio = _config_params_loader["LeggedRobotRLConfig"]["still_ratio"].as<float>();
  _robotConfig.enable_ankle_tau_mapping = _config_params_loader["LeggedRobotRLConfig"]["enable_ankle_tau_mapping"].as<bool>();

  _lpf_action = std::make_unique<LowPassFilter<Eigen::Matrix<float, 12, 1>>>(100, _config_params_loader["LeggedRobotRLConfig"]["cutoff_frequency"].as<float>());

  // joystick command scale
  _robotConfig.joy_linvX_scale = _config_params_loader["LeggedRobotRLConfig"]["joystick"]["linear_vX_scale"].as<float>();
  _robotConfig.joy_linvY_scale = _config_params_loader["LeggedRobotRLConfig"]["joystick"]["linear_vY_scale"].as<float>();
  _robotConfig.joy_omegaZ_scale = _config_params_loader["LeggedRobotRLConfig"]["joystick"]["linear_omegaZ_scale"].as<float>();
  _robotConfig.velx_bias = _config_params_loader["LeggedRobotRLConfig"]["joystick"]["velx_bias"].as<float>();
  _robotConfig.vely_bias = _config_params_loader["LeggedRobotRLConfig"]["joystick"]["vely_bias"].as<float>();
  _robotConfig.roll_bias = _config_params_loader["LeggedRobotRLConfig"]["joystick"]["roll_bias"].as<float>();
  _robotConfig.pitch_bias = _config_params_loader["LeggedRobotRLConfig"]["joystick"]["pitch_bias"].as<float>();

  // default joint angle
  _robotConfig.defaultJointPos.leg_l1_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_l1_joint"].as<float>();
  _robotConfig.defaultJointPos.leg_l2_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_l2_joint"].as<float>();
  _robotConfig.defaultJointPos.leg_l3_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_l3_joint"].as<float>();
  _robotConfig.defaultJointPos.leg_l4_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_l4_joint"].as<float>();
  _robotConfig.defaultJointPos.leg_l5_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_l5_joint"].as<float>();
  _robotConfig.defaultJointPos.leg_l6_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_l6_joint"].as<float>();
  _robotConfig.defaultJointPos.leg_r1_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_r1_joint"].as<float>();
  _robotConfig.defaultJointPos.leg_r2_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_r2_joint"].as<float>();
  _robotConfig.defaultJointPos.leg_r3_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_r3_joint"].as<float>();
  _robotConfig.defaultJointPos.leg_r4_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_r4_joint"].as<float>();
  _robotConfig.defaultJointPos.leg_r5_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_r5_joint"].as<float>();
  _robotConfig.defaultJointPos.leg_r6_joint = _config_params_loader["LeggedRobotRLConfig"]["init_state"]["default_joint_angle"]["leg_r6_joint"].as<float>();

  // control::stiffness
  _robotConfig.controlConfig.stiffness[jointNames[0]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_l1_joint"].as<float>();
  _robotConfig.controlConfig.stiffness[jointNames[1]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_l2_joint"].as<float>();
  _robotConfig.controlConfig.stiffness[jointNames[2]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_l3_joint"].as<float>();
  _robotConfig.controlConfig.stiffness[jointNames[3]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_l4_joint"].as<float>();
  _robotConfig.controlConfig.stiffness[jointNames[4]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_l5_joint"].as<float>();
  _robotConfig.controlConfig.stiffness[jointNames[5]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_l6_joint"].as<float>();

  _robotConfig.controlConfig.stiffness[jointNames[6]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_r1_joint"].as<float>();
  _robotConfig.controlConfig.stiffness[jointNames[7]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_r2_joint"].as<float>();
  _robotConfig.controlConfig.stiffness[jointNames[8]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_r3_joint"].as<float>();
  _robotConfig.controlConfig.stiffness[jointNames[9]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_r4_joint"].as<float>();
  _robotConfig.controlConfig.stiffness[jointNames[10]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_r5_joint"].as<float>();
  _robotConfig.controlConfig.stiffness[jointNames[11]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["stiffness"]["leg_r6_joint"].as<float>();

  // control::damping
  _robotConfig.controlConfig.damping[jointNames[0]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_l1_joint"].as<float>();
  _robotConfig.controlConfig.damping[jointNames[1]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_l2_joint"].as<float>();
  _robotConfig.controlConfig.damping[jointNames[2]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_l3_joint"].as<float>();
  _robotConfig.controlConfig.damping[jointNames[3]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_l4_joint"].as<float>();
  _robotConfig.controlConfig.damping[jointNames[4]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_l5_joint"].as<float>();
  _robotConfig.controlConfig.damping[jointNames[5]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_l6_joint"].as<float>();

  _robotConfig.controlConfig.damping[jointNames[6]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_r1_joint"].as<float>();
  _robotConfig.controlConfig.damping[jointNames[7]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_r2_joint"].as<float>();
  _robotConfig.controlConfig.damping[jointNames[8]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_r3_joint"].as<float>();
  _robotConfig.controlConfig.damping[jointNames[9]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_r4_joint"].as<float>();
  _robotConfig.controlConfig.damping[jointNames[10]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_r5_joint"].as<float>();
  _robotConfig.controlConfig.damping[jointNames[11]] = _config_params_loader["LeggedRobotRLConfig"]["control"]["damping"]["leg_r6_joint"].as<float>();

  // control::actionscale and control::decimation
  _robotConfig.controlConfig.actionScale = _config_params_loader["LeggedRobotRLConfig"]["control"]["action_scale"].as<float>();
  _robotConfig.controlConfig.decimation = _config_params_loader["LeggedRobotRLConfig"]["control"]["decimation"].as<float>();

  // control::gaitperiod
  _robotConfig.controlConfig.gait_period = _config_params_loader["LeggedRobotRLConfig"]["control"]["gait_period"].as<float>();

  // normalization::obs_scale
  _robotConfig.obsScales.linVel = _config_params_loader["LeggedRobotRLConfig"]["normalization"]["obs_scales"]["lin_vel"].as<float>();
  _robotConfig.obsScales.angVel = _config_params_loader["LeggedRobotRLConfig"]["normalization"]["obs_scales"]["ang_vel"].as<float>();
  _robotConfig.obsScales.dofPos = _config_params_loader["LeggedRobotRLConfig"]["normalization"]["obs_scales"]["dof_pos"].as<float>();
  _robotConfig.obsScales.dofVel = _config_params_loader["LeggedRobotRLConfig"]["normalization"]["obs_scales"]["dof_vel"].as<float>();
  _robotConfig.obsScales.quat = _config_params_loader["LeggedRobotRLConfig"]["normalization"]["obs_scales"]["quat"].as<float>();

  // normalization::clip_obs
  _robotConfig.clipObs = _config_params_loader["LeggedRobotRLConfig"]["normalization"]["clip_scales"]["clip_observations"].as<float>();

  // normalization::clip_act
  _robotConfig.clipActions = _config_params_loader["LeggedRobotRLConfig"]["normalization"]["clip_scales"]["clip_actions"].as<float>();

  // size
  _actionsSize = _config_params_loader["LeggedRobotRLConfig"]["size"]["actions_size"].as<float>();
  _observationSizeSingleFrame = _config_params_loader["LeggedRobotRLConfig"]["size"]["observations_size"].as<float>();
  _observationSizeFrameStack = _config_params_loader["LeggedRobotRLConfig"]["size"]["observations_frame_size"].as<int>();

  // print debug
  // printRobotConfig();

  int error = 0;

  _actions.resize(_actionsSize);
  _observations.resize(_observationSizeSingleFrame * _observationSizeFrameStack);
  std::fill(_observations.begin(), _observations.end(), 0.0f);
  _command.x = 0;
  _command.y = 0;
  _command.yaw = 0;

  std::vector<float> defaultJointPos{
      _robotConfig.defaultJointPos.leg_l1_joint, _robotConfig.defaultJointPos.leg_l2_joint, _robotConfig.defaultJointPos.leg_l3_joint,
      _robotConfig.defaultJointPos.leg_l4_joint, _robotConfig.defaultJointPos.leg_l5_joint, _robotConfig.defaultJointPos.leg_l6_joint,
      _robotConfig.defaultJointPos.leg_r1_joint, _robotConfig.defaultJointPos.leg_r2_joint, _robotConfig.defaultJointPos.leg_r3_joint,
      _robotConfig.defaultJointPos.leg_r4_joint, _robotConfig.defaultJointPos.leg_r5_joint, _robotConfig.defaultJointPos.leg_r6_joint};

  _lastActions.resize(_actionsSize);
  _lastActions.setZero();

  const int policyInputSize = _observationSizeFrameStack * _observationSizeSingleFrame;

  _observationHistoryBuffer.resize(policyInputSize);
  _defaultJointPos.resize(actuatedDofNum_);
  for (int i = 0; i < actuatedDofNum_; i++)
  {
    _defaultJointPos(i) = defaultJointPos[i];
  }

  return (error == 0);
}

void LearningBasedController::computeActions()
{
  // std::cout << "get into LearningBasedController::computeActions" << std::endl;

  // create input tensor object
  std::vector<Ort::Value> inputValues;
  inputValues.push_back(Ort::Value::CreateTensor<float>(_memoryInfo, _observations.data(), _observations.size(),
                                                        _inputShapes[0].data(), _inputShapes[0].size()));
  // run inference
  Ort::RunOptions runOptions;
  std::vector<Ort::Value> outputValues = _sessionPtr->Run(runOptions, _inputNames.data(), inputValues.data(), 1, _outputNames.data(), 1);

  for (int i = 0; i < _actionsSize; i++)
  {
    _actions[i] = *(outputValues[0].GetTensorMutableData<float>() + i);
  }

  Eigen::Matrix<float, 12, 1> act;
  for (int i = 0; i < _actionsSize; i++)
  {
    act(i, 0) = _actions[i];
  }
  _lpf_mlp_net_action_ = _lpf_action->Update(act);

  if (_robotConfig.enable_filter && _control_FSM_data->_desiredStateCommand->gait_type == 0 && still_flag)
  {
    for (int i = 0; i < _actionsSize; i++)
    {
      _actions[i] = _lpf_mlp_net_action_[i];
    }
  }
}
void LearningBasedController::updateBias()
{
  cmd_bias_vel_x = _control_FSM_data->_desiredStateCommand->cmd_vel_bias_x + _robotConfig.velx_bias;
  cmd_bias_vel_y = _control_FSM_data->_desiredStateCommand->cmd_vel_bias_y + _robotConfig.vely_bias;

  euler_roll_bias = _control_FSM_data->_desiredStateCommand->roll_bias + _robotConfig.roll_bias;
  euler_pitch_bias = _control_FSM_data->_desiredStateCommand->pitch_bias + _robotConfig.pitch_bias;

  // if (_control_FSM_data->_desiredStateCommand->bias_save_mode && (!_control_FSM_data->_desiredStateCommand->bias_save_mode_pre)) // bias save
  if (_control_FSM_data->_desiredStateCommand->bias_save_mode && save_yaml_config) // bias save
  {
    std::cout << "[Save the Bias Value to Yaml]" << std::endl;
    std::cout << "[Notice]: Though this value works immediately, it will be reloaded on the next launch of the control process" << std::endl;
    std::cout << "[current loaded cmd_bias_vel_x  ]: " << _robotConfig.velx_bias << " --> [new cmd_bias_vel_x to Yaml]: " << cmd_bias_vel_x << std::endl;
    std::cout << "[current loaded cmd_bias_vel_y  ]: " << _robotConfig.vely_bias << " --> [new cmd_bias_vel_y to Yaml]: " << cmd_bias_vel_y << std::endl;
    std::cout << "[current loaded euler_roll_bias ]: " << _robotConfig.roll_bias << " --> [new euler_roll_bias to Yaml]: " << euler_roll_bias << std::endl;
    std::cout << "[current loaded euler_pitch_bias]: " << _robotConfig.pitch_bias << " --> [new euler_pitch_bias to Yaml]: " << euler_pitch_bias << std::endl;

    if (_config_params_loader["LeggedRobotRLConfig"]["joystick"]["velx_bias"])
    {
      _config_params_loader["LeggedRobotRLConfig"]["joystick"]["velx_bias"] = std::to_string(cmd_bias_vel_x);
    }
    else
    {
      std::cerr << "Key [velx_bias] not found!" << std::endl;
    }

    if (_config_params_loader["LeggedRobotRLConfig"]["joystick"]["vely_bias"])
    {
      _config_params_loader["LeggedRobotRLConfig"]["joystick"]["vely_bias"] = std::to_string(cmd_bias_vel_y);
    }
    else
    {
      std::cerr << "Key [vely_bias] not found!" << std::endl;
    }

    if (_config_params_loader["LeggedRobotRLConfig"]["joystick"]["roll_bias"])
    {
      _config_params_loader["LeggedRobotRLConfig"]["joystick"]["roll_bias"] = std::to_string(euler_roll_bias);
    }
    else
    {
      std::cerr << "Key [roll_bias] not found!" << std::endl;
    }

    if (_config_params_loader["LeggedRobotRLConfig"]["joystick"]["pitch_bias"])
    {
      _config_params_loader["LeggedRobotRLConfig"]["joystick"]["pitch_bias"] = std::to_string(euler_pitch_bias);
    }
    else
    {
      std::cerr << "Key [roll_bias] not found!" << std::endl;
    }

    // Save the modified YAML back to the file
    std::ofstream fout("../config/zqsa01_rl.yaml");
    fout << _config_params_loader;

    fout.close();

    save_yaml_config = false;
  }
  else if (!_control_FSM_data->_desiredStateCommand->bias_save_mode)
  {
    save_yaml_config = true;
  }
  else
  {
    cmd_bias_vel_x = cmd_bias_vel_x;
    cmd_bias_vel_y = cmd_bias_vel_y;
    euler_roll_bias = euler_roll_bias;
    euler_pitch_bias = euler_pitch_bias;
  }
}

void LearningBasedController::computeObservation()
{
  //  command
  VecDynamic command(5);
  _command.x = (_control_FSM_data->_desiredStateCommand->leftAnalogStick[0] + cmd_bias_vel_x) * _robotConfig.joy_linvX_scale;
  _command.y = (_control_FSM_data->_desiredStateCommand->leftAnalogStick[1] + cmd_bias_vel_y) * _robotConfig.joy_linvY_scale;
  _command.yaw = _control_FSM_data->_desiredStateCommand->rightAnalogStick[0] * _robotConfig.joy_omegaZ_scale;

  double phase_cos = gait_phase_ / _robotConfig.controlConfig.gait_period;

  if (_control_FSM_data->_desiredStateCommand->gait_type == 0 && std::fmod(gait_phase_, _robotConfig.controlConfig.gait_period) < _robotConfig.still_ratio)
  {
    still_flag = true;
  }

  if (_control_FSM_data->_desiredStateCommand->gait_type == 0 && still_flag)
  {
    command[0] = 0.0;
    command[1] = 1.0;
    command[2] = 0.0;
    command[3] = 0.0;
    command[4] = 0.0;
  }
  else
  {
    command[0] = sin(2 * M_PI * phase_cos);
    command[1] = cos(2 * M_PI * phase_cos);
    command[2] = _command.x;
    command[3] = _command.y;
    command[4] = _command.yaw;
    still_flag = false;
  }

  // actions
  VecDynamic actions(_lastActions);
  Vec3_RL euler_bias(_stateCollection.torsoEulerXyz);

  euler_bias[0] += euler_roll_bias;
  euler_bias[1] += euler_pitch_bias;

  // std::cout<< "bias:"<<"roll:"<<_robotConfig.roll_bias<<" pitch:"<< _robotConfig.pitch_bias<<" velx:"<<_robotConfig.velx_bias<<" vely:"<<_robotConfig.vely_bias<<std::endl;
  LearningBasedRobotConfig::ObsScales &obsScales = _robotConfig.obsScales;
  MatDynamic commandScaler = Eigen::DiagonalMatrix<float, 3>(obsScales.linVel, obsScales.linVel, obsScales.angVel);

  VecDynamic proprioObs(_observationSizeSingleFrame);

  proprioObs << command,                                                 // 5
      (_stateCollection.jointPos - _defaultJointPos) * obsScales.dofPos, // 12
      _stateCollection.jointVel * obsScales.dofVel,                      // 12
      actions,                                                           // 12
      _stateCollection.torsoAngVel * obsScales.angVel,                   // 3
      euler_bias * obsScales.quat;                                       // 3

  // std::cout << "obs_command      : " << command.transpose() << std::endl;
  // std::cout << "obs_jointPos     : " << _stateCollection.jointPos.transpose() << std::endl;
  // std::cout << "obs_jointVel     : " << _stateCollection.jointVel.transpose() << std::endl;
  // std::cout << "obs_actions      : " << actions.transpose() << std::endl;
  // std::cout << "obs_baseAngVel   : " << _stateCollection.torsoAngVel.transpose() << std::endl;
  // std::cout << "obs_baseEulerXyz : " << _stateCollection.torsoEulerXyz.transpose() << std::endl;
  // std::cout << "Biased obs_baseEulerXyz : " << euler_bias.transpose() << std::endl;
  if (_isfirstRecObs)
  {
    for (int i = 29; i < 41; i++)
    {
      proprioObs(i, 0) = 0.0;
    }

    for (int i = 0; i < _observationSizeFrameStack; i++)
    {
      _observationHistoryBuffer.segment(i * _observationSizeSingleFrame, _observationSizeSingleFrame) = proprioObs.cast<float>();
    }
    _isfirstRecObs = false;
  }

  _observationHistoryBuffer.head(_observationHistoryBuffer.size() - _observationSizeSingleFrame) =
      _observationHistoryBuffer.tail(_observationHistoryBuffer.size() - _observationSizeSingleFrame);
  _observationHistoryBuffer.tail(_observationSizeSingleFrame) = proprioObs.cast<float>();

  for (int i = 0; i < (_observationSizeSingleFrame * _observationSizeFrameStack); i++)
  {
    _observations[i] = static_cast<float>(_observationHistoryBuffer[i]);
  }

  // clip observation range
  float obsMin = -_robotConfig.clipObs;
  float obsMax = _robotConfig.clipObs;
  std::transform(_observations.begin(), _observations.end(), _observations.begin(),
                 [obsMin, obsMax](float x)
                 { return std::max(obsMin, std::min(obsMax, x)); });
}
void LearningBasedController::updateRobotState()
{
  VecDynamic jointPos(_control_FSM_data->_legController->leg_control_data_biped.q.size()),
      jointVel(_control_FSM_data->_legController->leg_control_data_biped.qd.size());

  Quat_RL quat;
  Vec3_RL angularVel;

  for (int i = 0; i < actuatedDofNum_; i++)
  {
    _joint_datas.q[i] = _control_FSM_data->_legController->leg_control_data_biped.q[i];
    _joint_datas.qd[i] = _control_FSM_data->_legController->leg_control_data_biped.qd[i];
    _joint_datas.tau_state[i] = _control_FSM_data->_legController->leg_control_data_biped.tau_state[i];

    _motor_pos_state[i] = _joint_datas.q[i];
  }

  _ankle_joint_mapping->getForwardQVT(_joint_datas.q, _joint_datas.qd, _joint_datas.tau_state); // mapping annkle joint states from motor joints to serial joints

  for (int i = 0; i < actuatedDofNum_; i++)
  {
    jointPos(i) = _joint_datas.q[i];
    jointVel(i) = _joint_datas.qd[i];

    _joint_pos_state[i] = _joint_datas.q[i];
  }

  for (size_t i = 0; i < 4; ++i)
  {
    quat.coeffs()(i) = _control_FSM_data->_stateEstimator->getResult().ori_quat[i];
  }

  for (size_t i = 0; i < 3; ++i)
  {
    angularVel(i) = _control_FSM_data->_stateEstimator->getResult().omegaBody[i] * M_PI / 180; // original oemga(deg/s)
  }

  _stateCollection.jointPos = jointPos;
  _stateCollection.jointVel = jointVel;
  _stateCollection.torsoAngVel = angularVel;

  Vec3_RL gravityVector(0, 0, -1);
  Vec3_RL zyx = quatToZyx(quat);
  MatDynamic inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();
  _stateCollection.projectedGravity = inverseRot * gravityVector;
  _stateCollection.torsoEulerXyz = quatToXyz(quat);

  double control_dt = 1.0 / _robotConfig.control_frequency;
  gait_phase_ = _loopCount * control_dt;
}

void LearningBasedController::updateMotorCommand()
{
  if (_robotConfig.enable_ankle_tau_mapping)
  {
    /*tau cmd mapping*/
    _joint_datas.tau_ff[4] = _joint_datas.Kp[4] * (_joint_datas.q_des[4] - _joint_datas.q[4]) + _joint_datas.Kd[4] * (_joint_datas.qd_des[4] - _joint_datas.qd[4]);
    _joint_datas.tau_ff[5] = _joint_datas.Kp[5] * (_joint_datas.q_des[5] - _joint_datas.q[5]) + _joint_datas.Kd[5] * (_joint_datas.qd_des[5] - _joint_datas.qd[5]);
    _joint_datas.tau_ff[10] = _joint_datas.Kp[10] * (_joint_datas.q_des[10] - _joint_datas.q[10]) + _joint_datas.Kd[10] * (_joint_datas.qd_des[10] - _joint_datas.qd[10]);
    _joint_datas.tau_ff[11] = _joint_datas.Kp[11] * (_joint_datas.q_des[11] - _joint_datas.q[11]) + _joint_datas.Kd[11] * (_joint_datas.qd_des[11] - _joint_datas.qd[11]);

    _ankle_joint_mapping->getDecoupleQVT(_joint_datas.q, _joint_datas.qd, _joint_datas.tau_ff); // input the pos state for tau mapping

    for (int i = 0; i < actuatedDofNum_; i++)
    {

      _control_FSM_data->_legController->leg_control_data_biped.q_des[i] = _joint_datas.q_des[i];
      _control_FSM_data->_legController->leg_control_data_biped.qd_des[i] = _joint_datas.qd_des[i];
      _control_FSM_data->_legController->leg_control_data_biped.tau_ff[i] = _joint_datas.tau_ff[i];
      _control_FSM_data->_legController->leg_control_data_biped.kp[i] = _joint_datas.Kp[i];
      _control_FSM_data->_legController->leg_control_data_biped.kd[i] = _joint_datas.Kd[i];

      if (i == 4 || i == 5 || i == 10 || i == 11) // only tau_ff cmd for ankle joints
      {
        _control_FSM_data->_legController->leg_control_data_biped.kp[i] = 0.0;
        _control_FSM_data->_legController->leg_control_data_biped.kd[i] = 0.0;
      }
    }
  }
  else
  {
    /*pos cmd mapping */
    _ankle_joint_mapping->getDecoupleQVT(_joint_datas.q_des, _joint_datas.qd_des, _joint_datas.tau_ff); //

    for (int i = 0; i < 12; i++)
    {
      _control_FSM_data->_legController->leg_control_data_biped.q_des[i] = _joint_datas.q_des[i];
      _control_FSM_data->_legController->leg_control_data_biped.qd_des[i] = _joint_datas.qd_des[i];
      _control_FSM_data->_legController->leg_control_data_biped.tau_ff[i] = _joint_datas.tau_ff[i];
      _control_FSM_data->_legController->leg_control_data_biped.kp[i] = _joint_datas.Kp[i];
      _control_FSM_data->_legController->leg_control_data_biped.kd[i] = _joint_datas.Kd[i];
    }
    /*pos mapping test end*/
  }

  // std::cout << "q_des: " << _joint_datas.q_des.transpose() << std::endl;
  // std::cout << "qd_des: " << _joint_datas.qd_des.transpose() << std::endl;
  // std::cout << "tau_ff: " << _joint_datas.tau_ff.transpose() << std::endl;
  // std::cout << "Kp: " << _joint_datas.Kp.transpose() << std::endl;
  // std::cout << "Kd: " << _joint_datas.Kd.transpose() << std::endl;
}

void LearningBasedController::printRobotConfig()
{
  std::cout << "_robotConfig.control_frequency: " << _robotConfig.control_frequency << std::endl;

  std::cout << "_robotConfig.joy_linvX_scale: " << _robotConfig.joy_linvX_scale << std::endl;
  std::cout << "_robotConfig.joy_linvY_scale: " << _robotConfig.joy_linvY_scale << std::endl;
  std::cout << "_robotConfig.joy_omegaZ_scale: " << _robotConfig.joy_omegaZ_scale << std::endl;

  std::cout << "_robotConfig.velx_bias: " << _robotConfig.velx_bias << std::endl;
  std::cout << "_robotConfig.vely_bias: " << _robotConfig.vely_bias << std::endl;
  std::cout << "_robotConfig.roll_bias: " << _robotConfig.roll_bias << std::endl;
  std::cout << "_robotConfig.pitch_bias: " << _robotConfig.pitch_bias << std::endl;

  std::cout << "_robotConfig.defaultJointPos.leg_l1_joint: " << _robotConfig.defaultJointPos.leg_l1_joint << std::endl;
  std::cout << "_robotConfig.defaultJointPos.leg_l2_joint: " << _robotConfig.defaultJointPos.leg_l2_joint << std::endl;
  std::cout << "_robotConfig.defaultJointPos.leg_l3_joint: " << _robotConfig.defaultJointPos.leg_l3_joint << std::endl;
  std::cout << "_robotConfig.defaultJointPos.leg_l4_joint: " << _robotConfig.defaultJointPos.leg_l4_joint << std::endl;
  std::cout << "_robotConfig.defaultJointPos.leg_l5_joint: " << _robotConfig.defaultJointPos.leg_l5_joint << std::endl;
  std::cout << "_robotConfig.defaultJointPos.leg_l6_joint: " << _robotConfig.defaultJointPos.leg_l6_joint << std::endl;

  std::cout << "_robotConfig.defaultJointPos.leg_r1_joint: " << _robotConfig.defaultJointPos.leg_r1_joint << std::endl;
  std::cout << "_robotConfig.defaultJointPos.leg_r2_joint: " << _robotConfig.defaultJointPos.leg_r2_joint << std::endl;
  std::cout << "_robotConfig.defaultJointPos.leg_r3_joint: " << _robotConfig.defaultJointPos.leg_r3_joint << std::endl;
  std::cout << "_robotConfig.defaultJointPos.leg_r4_joint: " << _robotConfig.defaultJointPos.leg_r4_joint << std::endl;
  std::cout << "_robotConfig.defaultJointPos.leg_r5_joint: " << _robotConfig.defaultJointPos.leg_r5_joint << std::endl;
  std::cout << "_robotConfig.defaultJointPos.leg_r6_joint: " << _robotConfig.defaultJointPos.leg_r6_joint << std::endl;

  std::cout << "**********stiffness****************" << std::endl;
  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[0]]: " << _robotConfig.controlConfig.stiffness[jointNames[0]] << std::endl;
  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[1]]: " << _robotConfig.controlConfig.stiffness[jointNames[1]] << std::endl;
  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[2]]: " << _robotConfig.controlConfig.stiffness[jointNames[2]] << std::endl;
  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[3]]: " << _robotConfig.controlConfig.stiffness[jointNames[3]] << std::endl;
  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[4]]: " << _robotConfig.controlConfig.stiffness[jointNames[4]] << std::endl;
  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[5]]: " << _robotConfig.controlConfig.stiffness[jointNames[5]] << std::endl;

  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[6]]: " << _robotConfig.controlConfig.stiffness[jointNames[6]] << std::endl;
  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[7]]: " << _robotConfig.controlConfig.stiffness[jointNames[7]] << std::endl;
  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[8]]: " << _robotConfig.controlConfig.stiffness[jointNames[8]] << std::endl;
  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[9]]: " << _robotConfig.controlConfig.stiffness[jointNames[9]] << std::endl;
  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[10]]: " << _robotConfig.controlConfig.stiffness[jointNames[10]] << std::endl;
  std::cout << "_robotConfig.controlConfig.stiffness[jointNames[11]]: " << _robotConfig.controlConfig.stiffness[jointNames[11]] << std::endl;

  std::cout << "**********damping****************" << std::endl;
  std::cout << "_robotConfig.controlConfig.damping[jointNames[0]]: " << _robotConfig.controlConfig.damping[jointNames[0]] << std::endl;
  std::cout << "_robotConfig.controlConfig.damping[jointNames[1]]: " << _robotConfig.controlConfig.damping[jointNames[1]] << std::endl;
  std::cout << "_robotConfig.controlConfig.damping[jointNames[2]]: " << _robotConfig.controlConfig.damping[jointNames[2]] << std::endl;
  std::cout << "_robotConfig.controlConfig.damping[jointNames[3]]: " << _robotConfig.controlConfig.damping[jointNames[3]] << std::endl;
  std::cout << "_robotConfig.controlConfig.damping[jointNames[4]]: " << _robotConfig.controlConfig.damping[jointNames[4]] << std::endl;
  std::cout << "_robotConfig.controlConfig.damping[jointNames[5]]: " << _robotConfig.controlConfig.damping[jointNames[5]] << std::endl;

  std::cout << "_robotConfig.controlConfig.damping[jointNames[6]]: " << _robotConfig.controlConfig.damping[jointNames[6]] << std::endl;
  std::cout << "_robotConfig.controlConfig.damping[jointNames[7]]: " << _robotConfig.controlConfig.damping[jointNames[7]] << std::endl;
  std::cout << "_robotConfig.controlConfig.damping[jointNames[8]]: " << _robotConfig.controlConfig.damping[jointNames[8]] << std::endl;
  std::cout << "_robotConfig.controlConfig.damping[jointNames[9]]: " << _robotConfig.controlConfig.damping[jointNames[9]] << std::endl;
  std::cout << "_robotConfig.controlConfig.damping[jointNames[10]]: " << _robotConfig.controlConfig.damping[jointNames[10]] << std::endl;
  std::cout << "_robotConfig.controlConfig.damping[jointNames[11]]: " << _robotConfig.controlConfig.damping[jointNames[11]] << std::endl;

  std::cout << "_robotConfig.controlConfig.actionScale: " << _robotConfig.controlConfig.actionScale << std::endl;
  std::cout << "_robotConfig.controlConfig.decimation: " << _robotConfig.controlConfig.decimation << std::endl;
  std::cout << "_robotConfig.controlConfig.gait_period: " << _robotConfig.controlConfig.gait_period << std::endl;

  std::cout << "**********normization****************" << std::endl;
  std::cout << "_robotConfig.obsScales.linVel: " << _robotConfig.obsScales.linVel << std::endl;
  std::cout << "_robotConfig.obsScales.angVel: " << _robotConfig.obsScales.angVel << std::endl;
  std::cout << "_robotConfig.obsScales.dofPos: " << _robotConfig.obsScales.dofPos << std::endl;
  std::cout << "_robotConfig.obsScales.dofVel: " << _robotConfig.obsScales.dofVel << std::endl;
  std::cout << "_robotConfig.obsScales.quat: " << _robotConfig.obsScales.quat << std::endl;

  std::cout << "**********cliption****************" << std::endl;
  std::cout << "_robotConfig.clipObs: " << _robotConfig.clipObs << std::endl;
  std::cout << "_robotConfig.clipActions: " << _robotConfig.clipActions << std::endl;

  std::cout << "_observationSizeSingleFrame: " << _observationSizeSingleFrame << std::endl;
  std::cout << "_actionsSize: " << _actionsSize << std::endl;
}
