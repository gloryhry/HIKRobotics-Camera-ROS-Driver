# HIKROBOTICS工业相机ROS非官方驱动

[![ROS Noetic](https://github.com/gloryhry/HIKRobotics-Camera-ROS-Driver/actions/workflows/main.yml/badge.svg)](https://github.com/gloryhry/HIKRobotics-Camera-ROS-Driver/actions/workflows/main.yml)

海康机器人工业相机ROS驱动包，包含海康MVS-SDK v2.1.2静态库。支持参数化配置，支持关闭自动曝光时通过ROS-topic配置曝光时间。

仅通过MV-CA050-20GC相机测试，无法保证其他相机的适配性。

## Install

```bash

mkdir -p ~/hikrobotics_camera_ws/src
git clone https://github.com/gloryhry/HIKRobotics-Camera-ROS-Driver.git ~/hikrobotics_camera_ws/src/hik_camera_driver
cd ~/hikrobotics_camera_ws
catkin_make
```

## Launch run

```bash
source ./devel/setup.bash
roslaunch hik_camera_driver camera.launch
```

## Params

```yaml
# 相机名称
camera_name: camera
Camera:
  # 相机序列号
  serial_number: xxxxxxxxxxx
  # 相机内参文件 package://my_cameras/calibrations/${NAME}.yaml
  cam_info_url: package://hik_camera_driver/config/ost.yaml
  # 帧率
  frame_rate: 10.0

  # 触发模式
  Trigger: false
  # 触发源
  # 0:Line0
  # 1:Line1
  # 2:Line2
  # 3.Line3
  # 4:Counter0
  # 7:Software
  # 8:FrequencyConverter
  Tigger_line: 2
  # 触发上升沿、下降沿、高电平、低电平等
  # 0:RisingEdge
  # 1:FallingEdge
  # 2:LevelHigh
  # 3:LevelLow
  Trigger_action: 0
  # 触发延时 ≥0.0 ，单位 us
  Trigger_delay: 0.0

  # 自动曝光
  # 0:Off
  # 1:Once
  # 2:Continuous
  Exposure: 2
  # 曝光时间 ≥0.0 ，单位 us
  Exposure_time: 5000
  # 自动曝光时间上限
  ExposureTimeUp: 10000
  # 自动曝光时间下限
  ExposureTimeLow: 100

  # 自动增益
  # 0:Off
  # 1:Once
  # 2:Continuous
  Gain_mode: 2
  # 增益值 ≥0.0 ，单位 dB
  Gain_value: 0.0

  # 数字偏移使能
  Digital_shift_mode: true
  # 数字偏移调节 ≥ 0.0 
  Digital_shift: 3.0
  # 亮度 0 <= L <= 255
  brightneess: 80

  # Gamma 使能
  GammaEnable: true
  # 伽马调节
  Gamma_value: 1.0
  # Gamma 选择
  # 1:User
  # 2:sRGB
  Gamma_selector: 1
```
