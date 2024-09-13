# ENGINEAI HUMANOID
---
Locomotion control algorithm for bipedal robots of Shenzhen EngineAI Robotics Technology Co., Ltd. based on the MIT locomotion control algorithm [Cheetah Software](https://github.com/mit-biomimetics/Cheetah-Software), mainly includes modules as below:
- common
  - robot modelling algorithm
  - close chain mapping algorithm 
  - ...
- config
  - normal parameters of robot configuration
- lcm-types
  - lcm datas for logging and debugging
- robot
- scripts
- third-party
- user
  - deploy algorithm of RL(Done)
  - planning algorithm module of model based control (ongoing)
  - controlling algorithm module of model based control (ongoing)
  - user's algorithm developing space

The simulation environment for sim2sim is currently offered with our RL training repository [engineai_legged_gym](https://github.com/engineai-robotics/engineai_legged_gym), and will be transferred to this repository laterly.


Additionally, framework of this repository is designed to combine model based algorithm and learning based algorithm. And the model based part will be opened in the future.

# Contents
- [ENGINEAI HUMANOID](#engineai-humanoid)
- [Contents](#contents)
- [1. Deploy Process](#1-deploy-process)
  - [1.1 Dependencies Installation](#11-dependencies-installation)
    - [1.1.1 Cmake](#111-cmake)
    - [1.1.2 Eigen](#112-eigen)
    - [1.1.3 Yaml](#113-yaml)
    - [1.1.4 Lcm](#114-lcm)
    - [1.1.5 Ros](#115-ros)
    - [1.1.6 Other dependency for onnx](#116-other-dependency-for-onnx)
    - [1.1.7 onnx](#117-onnx)
    - [1.1.8 libmotor](#118-libmotor)
  - [1.2  Compilation](#12--compilation)
  - [1.3 Real Robot Deployment](#13-real-robot-deployment)
    - [1.3.1 Control the robot with the default deployment](#131-control-the-robot-with-the-default-deployment)
      - [1.3.1.1 Start robot](#1311-start-robot)
      - [1.3.1.2 Joystick control](#1312-joystick-control)
    - [1.3.2 Control the robot with your configuration and policy](#132-control-the-robot-with-your-configuration-and-policy)
- [2. Open-source Requirements](#2-open-source-requirements)
  - [2.1 Code Format](#21-code-format)
  - [2.2 Code Style](#22-code-style)
  - [2.3 Code Submit](#23-code-submit)

# 1. Deploy Process
The control algorithm is mainly used for reinforcement learning (RL) deployment on real robot. This repository works with our RL algorithm [engineai_legged_gym](https://github.com/engineai-robotics/engineai_legged_gym).

In order to deploy the trained policy to the robot, there are two devices needed. One which is called local developing device( such as a local PC) is  used for compiling this repository to generate the library and executable targets, and the other one is the so called mainboard NaZha which is already equipped in our robot. 

## 1.1 Dependencies Installation

The mainboard (NeZha) is already configured properly, it is not needed to do the following configuraions. We suggest you do not change the default development configuration, in case of failling to run your policy trained with our RL framework  [engineai_legged_gym](https://github.com/engineai-robotics/engineai_legged_gym)

If you are interested in the configuration process, and want to do it by yourself, the following dependencies with specified version are suggested. 

Again, WE SUGGEST YOU DO NOT CHANGE THE DEFAULT DEVELOPMENT ENVIRONMENT ON BOARD.

For your local developping device, the following dependencies with the specified version are suggested. If you has any problem with the compiling or running process, remember to check the versions of each depencency.

- **operating system**: ubuntu 20.04

### 1.1.1 Cmake

Since onnx is used to update the RL algorithm, cmake version must above 3.26

Download zip and extrac it
```
$ wget https://cmake.org/files/v3.28/cmake-3.28.5.zip
```
Move the file to your destination and install cmake

```
$ cd cmake-3.28.5
$ chmod 777 ./configure
$ ./configure
$ make
$ sudo make install
```
If you already installed a lower version cmake, replace it with following command

```
$ sudo update-alternatives --install /usr/bin/cmake cmake /usr/local/bin/cmake 1 --force
```
Check your cmake version

```
$ cmake --version
```
### 1.1.2 Eigen

Install eigen with apt install
```
$ sudo apt install libeigen3-dev
```
Check whether a folder named "eigen" exist in /usr/local/include/, if not install eigen with make install as following.
```
$ git clone https://gitlab.com/libeigen/eigen.git
$ cd eigen
$ git checkout 3.3.7
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
```

### 1.1.3 Yaml

Install yaml based on make install
```
$ git clone https://github.com/jbeder/yaml-cpp.git
$ cd yaml-cpp
$ git checkout yaml-cpp-0.6.3
$ mkdir build
$ cd build
$ cmake -DYAML_BUILD_SHARED_LIBS=ON ..
$ make -j4
$ sudo make install
```

### 1.1.4 Lcm
Install the software according to the steps on the [LCM official website](https://lcm-proj.github.io/lcm/). 
lcm 1.4.0 and 1.5.0 are tested, other versions are not guaranteed.
```
$ git clone https://github.com/lcm-proj/lcm.git
$ cd lcm
$ git checkeout v1.5.0
$ mkdir build
$ cd build
$ cmake -DLCM_ENABLE_JAVA=ON ..
$ make
$ sudo make install
```
you may need some commands as following
```
$ sudo apt install build-essential libglib2.0-dev
$ sudo apt install default-jdk python-all-dev liblua5.1-dev golang doxygen
$ sudo apt install openjdk-11-jdk
```
### 1.1.5 Ros
Ros is used for onnx and imu library. Follow [Ros offical website](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ros-neotic on your computer.

You may used some commands as below
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
$ sudo apt install ros-noetic-desktop-full
$ source /opt/ros/noetic/setup.bash
$ roscore
```

### 1.1.6 Other dependency for onnx
```
$ sudo apt-get update
$ sudo apt-get install -y libprotobuf-dev protobuf-compiler
```

### 1.1.7 onnx
Clone onnxruntime from [microsoft github](https://github.com/microsoft/onnxruntime)
```
$ git clone https://github.com/microsoft/onnxruntime
$ cd onnxruntime
$ git checkout v1.19.0
$ ./build.sh --config Release --build_shared_lib --parallel --allow_running_as_root
$ cd onnxruntime/build/Linux/Release/
$ sudo make install
```

onnxruntime version 1.19.0 is suggested. If you would like to use the other version, any conflict generated during the complilation process should be solved by yourself.

### 1.1.8 libmotor
In the directory of ```projectpath/dep-pkgs/..```, you can get the motor API of our robot, and install it on your local developing device with the following command.
```
sudo dpkg -i motor-mcu_1.0.8_amd64.deb
``` 

To get deep knowledge of how to use the motor libs, please refer to the ```README.md```  in that directroy and the source code of this repository. 

## 1.2  Compilation

```
$ git clone https://github.com/engineai-robotics/engineai_humanoid
$ cd engineai_humanoid/EngineAI_Controller/
$ mkdir build
$ cd build
$ cmake ..
$ make -j
$ make install
```

And now, you can find a folder named **EngineAI_Humanoid** within the build directory. And the folder contains all the needed install targets.

## 1.3 Real Robot Deployment
### 1.3.1 Control the robot with the default deployment
#### 1.3.1.1 Start robot
The robot is defaulty configured auto-start of the control process. Try to start the robot according to the following steps:
- press the power button, check whether the power bank is full. 
  - If fewer than two lights are enlightened, you need to recharge the power bank.  
  - If the power is enough, insert the power bank into the robot back.
- first shortly press the power button, and then continuely press it. Please release the button when it is enlightened.
- waiting about 30 seconds for the control process to launch

After the above operation, you can now use the joystick to control the robot.

#### 1.3.1.2 Joystick control
Please power on the leg motors with our ESTOP device before you operate the joystick. And press the ESTOP device for once, if you hear the sound "Di Di" for twice, then the motors are powered on. If the sound is heard once, the motors are powered off.

- Press `LB + back` to disable motors (no motor torque)
- Press `LB + start` to enable motors (no motor torque)
- Press `LB + B` to enter bented-leg stand mode
- Press `LB + A` to enter straight-leg stand mode
- Press `LB + X` to enter RL locomotion mode (can only transist from straight stand mode)
  -  Press `A` to alternate standing and walking of RL mode 

Normally, the operation sequency is: power on->`LB + back`->`LB + start`->`LB + B` or `LB + A`-> make robot stand by itself ->`X`->`LB + A` -> lift robot up -> `LB + start` ->`LB + back`-> shut down

**Note that: When robot stands or walks by itsefly, only use `LB + RB`, `LB + start` or `LB + back`in emergency, robot may fall down after that.**

### 1.3.2 Control the robot with your configuration and policy
- Connect the robot to your computer with an Ethernet cable
- Change the network of your computer to static ip，Address:192.168.0.100, Network:255.255.255.0, Gateway:192.168.0.1
- Use `ifconfig` to find your network device, for example `enp0s25`, use folloing commands to set lcm port
```
$ sudo ifconfig enp0s25 multicast`
$ sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp0s25
```
- Add the following contents into your bashrc for decoding lcm types
  ``` 
  export CLASSPATH=$CLASSPATH:your_path/EngineAI_Humanoid/EnginrAI_Controller/lcm-types/my_types.java
  ```
- ssh into the robot`ssh user@192.168.0.163`, password is `1`
- Mount the robot computer disk to your own computer and copy the EngineAI_Humanoid folder to robot computer `/home/user/`
- Stop the default control process which is auto-started `sudo pkill EngineAI_Controller`
- Copy the policy file zqsa01_policy.onnx generated by our RL trainning framework to the following path
  ```
  EngineAI_Humanoid/install/policy/zqsa01/
  ```
- Change K_p, K_d and the other parameters as needed in `EngineAI_Humanoid/install/config/zqsa01_rl.yaml`
- Remember to make a copy of the default zqsa01_policy.onnx and zqsa01_rl.yaml file so that to keep the default locomotion skills
- Run `./run_biped.sh` on the robot computer
- Run `lcm-spy` on your local computer to check the joint poses and the other datas

# 2. Open-source Requirements
## 2.1 Code Format
Coding format is defined by the file '.clang-format', how to use:
```
$ sudo apt install clang-format # install clang-format
$ clang-format -i file_to_format.cpp # specify which file to format
$ find . -regex '.*\.\(cpp\|hpp\|c\|h\)' -exec clang-format -style=file -i {} \; # format all the files ending with .cpp, .hpp, .c and .h under the current directory and its sub-directories
```

## 2.2 Code Submit
For easier tracking of modification, please follow the submission format below:
```
[Fix/New/Modify]: summary

1. commit message

Change-ID
Signed-off-by: xxx <xxx@company.com>
```
To be noted:
- suggest using the command 'git commit -s' to submit so that signed-off information can be automatically generated;
- submit types can only be chosen from three: Fix(bug fixing), New(new feature)，Modify(code modify);
- please describe more details in the commit message and avoid repeating the summary line.

We also provided a commit template for your convenience：
```
$ git config --global commit.template ./.commit-template
```
