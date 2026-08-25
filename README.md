# ESP App Firmware

基于 ESP-IDF 编译的 espapp 控制器固件。

## 功能简介

- 接收期望位移，输出力和力矩
- 基于 micro-ROS 实现与上位机通信（话题：`/espapp/input`、`/espapp/output`）

## 实现细节

- 预设时间收敛观测器
- 预设时间收敛导引律
- LSRL-PID 控制器

## 使用方式

- 加载 esp-idf 环境并编译烧录

## 测试环境

- WSL Ubuntu 26.04
- ESP-IDF `v5.5.4`
- micro-ROS Humble
- ESP32-P4
