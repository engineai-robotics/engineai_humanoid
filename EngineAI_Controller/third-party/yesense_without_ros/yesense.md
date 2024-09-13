
## 1. 使用

```shell
roslaunch yesense_imu test_yesense.launch
```

```html
<launch>

  <node pkg="yesense_imu" type="yesense_node" name="yesense_node"  output = "screen" >
    <param name="yesense_port"      type="string" value="/dev/ttyACM0"/>
    <param name="yesense_baudrate"  type="int"    value="460800"/>
  </node>

  <!-- <node pkg="rviz" type="rviz" name="rviz" args="-d $(find yesense_imu)/rviz/demo.rviz" required="true"/> -->

</launch>
```



其中，**yesense_port**表示设备串口号， **yesense_baudrate**表示设备波特率



## 2. 参数设置指令表

### 2.1 查询

#### 2.1.1 产品信息相关查询(0x00)

1. 查询软件版本号

```shell
rostopic pub /production_query std_msgs/Int8 "data: 1" 
```

2. 查询产品型号

```shell
rostopic pub /production_query std_msgs/Int8 "data: 2" 
```

#### 2.1.2 查询波特率(0x02)

```shell
rostopic pub /baudrate_query std_msgs/Empty "{}"
```

#### 2.1.3 查询输出频率(0x03)

```
rostopic pub /freequency_query std_msgs/Empty "{}"
```

#### 2.1.4 查询输出内容(0x04)

```
rostopic pub /output_content_query std_msgs/Empty "{}"
```

#### 2.1.5 查询标准参数（0x05）

1. 查询陀螺用户零偏

   ```
   rostopic pub /standard_param_query std_msgs/UInt8 "data: 1"
   ```

2. 读取静态阈值

   ```
   rostopic pub /standard_param_query std_msgs/UInt8 "data: 2"
   ```

#### 2.1.6 查询算法模式(0x4d)

1. 算法模式

```
rostopic pub /mode_query  std_msgs/UInt8 "data: 1"
```

2. 动态模式查询

```
rostopic pub /mode_query  std_msgs/UInt8 "data: 2"
```

### 2.2 配置掉电不保存

#### 2.2.1 复位所有参数(0x01)

已弃用

#### 2.2.3 设置波特率(0x02)

最高位为1，则参数设置掉电不保存，最高位为0，则参数设置掉电保存。低4位为设置的波特率代表的数值。

1. 设置波特率为9600

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x81"
   ```

2. 设置波特率为38400

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x82"
   ```

3. 设置波特率为115200

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x83"
   ```

4. 设置波特率为460800

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x84"
   ```

5. 设置波特率为921600

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x85"
   ```

6. 设置波特率为19200

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x86"
   ```

7. 设置波特率为57600

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x87"
   ```

8. 设置波特率为76800

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x88"
   ```

9. 设置波特率为230400

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x89"
   ```

#### 2.2.3 设置输出频率(0x03)

最高位为1，则参数设置掉电不保存，最高位为0，则参数设置掉电保存。低4位为设置的输出频率代表的数值。

1. 设置输出频率为1 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x81" 
   ```

2. 设置输出频率为2 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x82"
   ```

3. 设置输出频率为5 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x83"
   ```

4. 设置输出频率为10 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x84"
   ```

5. 设置输出频率为20 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x85"
   ```

6. 设置输出频率为25 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x86"
   ```

7. 设置输出频率为50 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x87"
   ```

8. 设置输出频率为100 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x88"
   ```

9. 设置输出频率为200 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x89"
   ```

#### 2.2.4 设置输出内容(0x04)

1. 全部不输出

   ```
   rostopic pub /output_content_setting std_msgs/UInt8 "data: 0x80"
   ```

2. 加计、陀螺、磁、欧拉、四元素

   ```
   rostopic pub /output_content_setting std_msgs/UInt8 "data: 0x81"
   ```

3. 位置、速度、UTC、加计、陀螺、磁、欧拉、四元素

   ```
   rostopic pub /output_content_setting std_msgs/UInt8 "data: 0x82"
   ```

#### 2.2.5 标准参数设置(0x05)

1. 陀螺用户零偏差值

   ```
   rostopic pub /standard_param_setting std_msgs/UInt8 "data: 0x80"
   ```

2. 嵌入式零偏校准

    ```
    rostopic pub /mode_setting std_msgs/UInt8 "data: 0x8A"
    ```


### 2.3 配置掉电保存

#### 2.3.1 设置波特率(0x02)

最高位为1，则参数设置掉电不保存，最高位为0，则参数设置掉电保存。低4位为设置的波特率代表的数值。

1. 设置波特率为9600

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x01"
   ```

2. 设置波特率为38400

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x02"
   ```

3. 设置波特率为115200

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x03"
   ```

4. 设置波特率为460800

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x04"
   ```

5. 设置波特率为921600

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x05"
   ```

6. 设置波特率为19200

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x06"
   ```

7. 设置波特率为57600

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x07"
   ```

8. 设置波特率为76800

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x08"
   ```

9. 设置波特率为230400

   ```
   rostopic pub /baudrate_setting std_msgs/UInt8 "data: 0x09"
   ```

#### 2.3.3 设置输出频率(0x03)

最高位为1，则参数设置掉电不保存，最高位为0，则参数设置掉电保存。低4位为设置的输出频率代表的数值。

1. 设置输出频率为1 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x01" 
   ```

2. 设置输出频率为2 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x02"
   ```

3. 设置输出频率为5 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x03"
   ```

4. 设置输出频率为10 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x04"
   ```

5. 设置输出频率为20 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x05"
   ```

6. 设置输出频率为25 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x06"
   ```

7. 设置输出频率为50 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x07"
   ```

8. 设置输出频率为100 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x08"
   ```

9. 设置输出频率为200 HZ

   ```
   rostopic pub /freequency_setting std_msgs/UInt8 "data: 0x09"
   ```

#### 2.3.4 设置输出内容(0x04)

最高位为1，则参数设置掉电不保存，最高位为0，则参数设置掉电保存。低4位为设置的波特率代表的数值。

1. 全部不输出

   ```
   rostopic pub /output_content_setting std_msgs/UInt8 "data: 0x00"
   ```

2. 加计、陀螺、磁、欧拉、四元素

   ```
   rostopic pub /output_content_setting std_msgs/UInt8 "data: 0x01"
   ```

3. 位置、速度、UTC、加计、陀螺、磁、欧拉、四元素

   ```
   rostopic pub /output_content_setting std_msgs/UInt8 "data: 0x02"
   ```

#### 2.3.5 标准参数设置(0x05)

最高位为1，则参数设置掉电不保存，最高位为0，则参数设置掉电保存。低4位为设置的波特率代表的数值。

1. 姿态角设置为0

   ```
   rostopic pub /standard_param_setting std_msgs/UInt8 "data: 0x01"
   ```

2. 航向角设置为零

   ```
   rostopic pub /standard_param_setting std_msgs/UInt8 "data: 0x02"
   ```

3. 陀螺用户零偏差值

   ```
   rostopic pub /standard_param_setting std_msgs/UInt8 "data: 0x03"
   ```

#### 2.3.6 设置算法模式(0x4d)
1. 嵌入式零偏校准

    ```
    rostopic pub /mode_setting std_msgs/UInt8 "data: 0x0A"
    ```

