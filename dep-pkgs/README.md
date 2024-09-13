# MCU MOTORS API


### int motor_init(void)
- initialize spi device（/dev/spidev1.0,/dev/spidev1.1）and memory
- if succeeds, return 0
- if fails, return -1

---
### void motor_uninit(void)
- free memory and resources related to spi operation

---
### motor_command_t *get_motor_cmd(void)
- get the received motor command from the motor mcu

---
### motor_data_t *get_motor_data(void)
- get the current motor stats from motor mcu

---
### int set_motor_cmd(motor_command_t *cmd)
- send the motor command to motor mcu

---
### int set_motor_enable_cmd(int enable, int index)
- enable = true, enable motors
- enable = false, disable motors
- index
  - 0 ~ 5
    - representing the joints of left leg
    - joint roll, yaw, pitch of left thigh, left knee joint, left inner ankle joint, and left outside ankle joint
  - 6 ~ 11
    - representing the joints of right leg
    - joint roll, yaw, pitch of right thigh, right knee joint, right inner ankle joint, and right outside ankle joint
  - you can enable a specific motor with the corresponding index number. If you set index to 12, it will enable all the motors. 

---

### int set_motor_zero_cmd(int index)
- joint zero calibration 
- index
  - 11 ~ 16
    - to set the zero offset for the joints of left leg
  - 21 ~ 23 
    - to set the zero offset for joints of the right leg
---
[NOTICE]: All the motors have already been cablibrated, you do not need to do it again unless the zero offset loses. If unfurtunately it happens, you need to follow the directions below to do zero calibration. 

1. insert the pin to each joint
2. ssh to the mainboard(NeZha) ```ssh user@192.168.0.163```  ```passward: 1```
3. zero calibration (write the zero offset to the mcu flash)
   - zero calibration for all the motors
       ```
       sudo motor_zero 0
       ```
   - zero caliobration for a specific motor
       ```
       sudo motor_zero XX 1
       ```
     - XX means the index of the motor
       - 11 ~ 16: joints of left leg
       - 21 ~ 26: joints of right leg
4. power off the motor and power it on angin to make zero calibration takes effect.
