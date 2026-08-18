# ASV Controller Firmware

基于 ESP-IDF 编译的 ASV 控制器固件。

## 功能简介

- 接收 ASV 坐标系下的期望位移，输出力和力矩
- 基于 micro-ROS 实现与上位机通信

## 实现细节

- 预设时间收敛观测器
- 预设时间收敛导引律
- LSRL-PID 控制器

## 项目结构

```
asv-esp32-firmware/
├── components/core/          # ESP-IDF 固件核心组件
├── extra_ros_packages/
│   └── interfaces/           # micro-ROS 自定义消息包
├── main/                     # 应用入口
└── app-colcon.meta           # micro-ROS colcon 配置
```

## 编译

加载 ESP-IDF 环境（推荐使用 `~/idf55_microros_env.sh`，并保持单一 Python 环境）：

```bash
source ~/idf55_microros_env.sh
cd ~/asv-esp32-firmware
pip install -r requirements-microros.txt
idf.py build
```

修改 `extra_ros_packages/` 中的消息后，需重新生成 micro-ROS 头文件：

```bash
idf.py clean-microros
idf.py build
```

若出现 Python 环境不一致，执行 `idf.py fullclean` 后重新编译。

## 测试环境

- WSL Ubuntu 26.04
- ESP-IDF `v5.5.4`
- micro-ROS Humble
- ESP32-P4
