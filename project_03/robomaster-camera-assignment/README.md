# HIKROBOT Camera ROS 2 Package

本功能包基于海康机器人 (HIKROBOT) MVS SDK，在 Ubuntu 22.04 / ROS 2 Humble 环境下实现了相机的设备发现、图像采集发布、动态参数配置及断线重连功能。

## 1. 环境准备与 SDK 安装

### 1.1 安装 MVS SDK
1. 前往 [海康机器人官网](https://www.hikrobotics.com/cn/machinevision/service/download/?module=0) 下载 **Linux 版本 MVS 软件包** (选择 Ubuntu 22.04 对应的版本)。
2. 解压下载的文件，进入解压后的目录。
3. 运行安装脚本（需要 root 权限）：
   ```bash
   sudo ./MVS.sh