# tianracer_autodrive

tianracer_autodrive 是一个针对 Tianbot 平台开发的 ROS2GO 的 Tianracer T110 无人车项目的自动驾驶代码库，目前包含寻墙算法自动驾驶控制、传感器数据处理、路径规划等功能模块。  
本项目由 星环队 开源，旨在为本项目赛队和开发者提供一个自动驾驶算法库，便于后期迭代升级。

## 当前版本
- **稳定版本**: v0.4
- **可用版本**: v0.0-0.4
- **开发中**: v0.5

## 功能特点
- 支持激光雷达的数据采集与处理
- 基于 寻墙算法 的路径识别与规划
- 模块化设计，方便二次开发与功能扩展

## 硬件环境
- **底盘**: TianRacer T110 无人车底盘
- **主控**: Jetson Nano
- **传感器**: LiDAR, USB camera, RGBD camera(optional), GPS(optional)
- **电源**: 3S 锂电池

## 软件环境
- 操作系统: ROS 2.0 / ROS2GO
- 依赖：
  - Python >= 3.8
  - gmapping / cartographer / hector slam
  - Rviz

更多软硬件相关内容，可以查阅 tianbot/tianracer

tianracer_autodrive is an autonomous driving codebase developed for the Tianbot platform’s ROS2GO Tianracer T110 unmanned vehicle.
It currently includes wall-following algorithm-based autonomous driving control, sensor data processing, path planning, and other functional modules.
This project is open-sourced by Team StarRing, aiming to provide both the team and developers with a reusable autonomous driving algorithm library for future iterations and upgrades.

## Versions
- **Stable Version**: v0.4
- **Usable Version**: v0.0-0.4
- **Developing Version**: v0.5

## Features
- Supports LiDAR data acquisition and processing
- Path detection and planning based on the wall-following algorithm
- Modular design for easy secondary development and feature expansion

## Hardware
- **Chassis**: TianRacer T110 unmanned vehicle chassis
- **Main Controller**: Jetson Nano
- **Sensors**: LiDAR, USB camera, RGBD camera (optional), GPS (optional)
- **Power Supply**: 3S LiPo battery

## Software
- Operating System: ROS 2.0 / ROS2GO
- Dependencies:
  - Python >= 3.8
  - gmapping / cartographer / hector slam
  - Rviz

For more hardware and software details, please refer to tianbot/tianracer.

由星环队 Zephyr 开源
最近一次更新 Aug./13/2025 v0.1

Open-sourced by Team StarRing - Zephyr
Last updated: Aug./13/2025 v0.1
