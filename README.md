# 面向 ASV 仿真的硬件在环实验平台 - ESP32端

基于 ESP-IDF 编译的 espapp 固件。

## 功能接口

- 按照固定的控制频率，接收 ASV 在 BodyFrame 下的二维期望位移，输出力和力矩
- 基于 micro-ROS 实现与上位机 Jetson Orin Nano 的通信（订阅话题：`/espapp/input`， 发布话题： `/espapp/output`）
- 输入：二维期望位移、 ASV 自身纵向速度、 ASV 自身角速度（后两者在实际中可通过 IMU 传感器获得）
- 输出：力、力矩

## 实现细节

- 接收到某一时刻的二维期望位移后，基于预设时间收敛观测器，计算二维期望位移轨迹的速度
- 根据二维期望位移轨迹的速度，基于预设时间收敛导引律，计算 ASV 的期望纵向速度改变量和航向角改变量
- 根据 ASV 的期望纵向速度改变量和航向角改变量，基于 LSRL-PID 控制器，计算期望力和力矩

## 测试环境

- WSL Ubuntu 26.04
- ESP-IDF `v5.5.4`
- micro-ROS Humble
- ESP32-P4
