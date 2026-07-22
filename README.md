# ASV Controller Firmware

基于 ESP-IDF 和 micro-ROS 的 ASV 控制器固件。

该固件运行于 ESP32-P4，负责接收决策指令、执行控制算法，并发布控制结果数据。

## 功能

- micro-ROS 通信
- ASV 控制输入与推力指令处理
- 控制器与状态估计
- UART 通信支持
- 自定义 ROS 2 消息接口

## 环境要求

- Ubuntu
- ESP-IDF `v5.5.4`
- Python（由 ESP-IDF 管理）
- ROS 2 Humble（用于 micro-ROS 接口开发）
- ESP32-P4 开发板