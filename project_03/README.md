# MVS 相机 SDK 的 ROS 2 封装

本项目基于海康机器人 MVS SDK，在 ROS 2 Humble (Ubuntu 22.04) 环境下开发了一个可复用的 C++ 相机功能包。实现了设备发现、图像采集与发布、动态参数配置、断线重连及资源管理等功能。

## 1. 开发环境

- **操作系统**: Ubuntu 22.04.5 LTS @ Linux 6.18.40.1-microsoft-standard-WSL2
- **ROS 版本**: ROS 2 Humble Hawksbill
- **硬件**: 海康机器人 USB 3.0 或 GigE 工业相机
- **依赖**: 海康机器人 MVS SDK (Linux_86_64)

## 2. 安装与配置

### 2.1 安装 MVS SDK
1. 前往 [海康机器人官网](https://www.hikrobotics.com/cn/machinevision/service/download?module=0) 下载对应平台的 MVS SDK。
2. 解压并运行安装脚本（通常需要 `sudo` 权限）：
   ```bash
   tar -xzf MVS-5.1.0_Linux_x86_64_20260909.zip
   cd MVS-5.1.0_Linux_x86_64_20260909
   sudo ./MVS.sh install
   ```
3. **重要**：安装完成后，需配置 udev 规则以赋予非 root 用户访问 USB 相机的权限（安装脚本通常会自动处理，若无效请手动执行 `sudo sh /opt/MVS/bin/usb_install.sh`）。

### 2.2 安装 ROS 依赖
在工作空间根目录下执行：
```bash
cd ~/RoboMaster/XJTU_RMAC # 替换为工作空间路径
rosdep install --from-paths src --ignore-src -r -y
```

### 2.3 WSL2 环境下的相机连接 (可选)
如果您在 WSL2 中开发，请按以下方式连接相机：
- **USB 相机**: 使用 `usbipd-win` 工具将 Windows 宿主机的 USB 设备映射到 WSL。
  ```powershell
  # Windows PowerShell (管理员)
  usbipd list
  usbipd bind --busid <BUSID>
  usbipd attach --wsl --busid <BUSID>
  ```
- **GigE 相机**: 建议在 Windows 用户目录下创建 `.wslconfig` 文件并启用镜像网络模式，以确保 WSL 能访问局域网相机。
  ```ini
  [wsl2]
  networkingMode=mirrored
  ```

## 3. 编译与运行

### 3.1 编译
```bash
cd ~/RoboMaster/XJTU_RMAC
colcon build --symlink-install --packages-select hikrobot_camera
source install/setup.bash
```

### 3.2 运行
使用提供的 Launch 文件启动节点：
```bash
ros2 launch hikrobot_camera camera.launch.py
```
若需指定相机 IP 或序列号，可通过命令行参数覆盖：
```bash
ros2 launch hikrobot_camera camera.launch.py camera_ip:=192.168.1.64
# 或
ros2 launch hikrobot_camera camera.launch.py camera_sn:=YOUR_CAMERA_SN
```

## 4. 参数说明

节点支持通过 ROS 2 参数服务器动态配置相机参数。参数定义位于 `config/camera.yaml`。

| 参数名 | 类型 | 默认值 | 单位/说明 | 动态修改限制 |
| :--- | :--- | :--- | :--- | :--- |
| `camera_ip` | string | `""` | GigE 相机 IP 地址。为空时自动发现。 | 启动时生效 |
| `camera_sn` | string | `""` | 相机序列号。为空时自动发现。 | 启动时生效 |
| `image_topic` | string | `/image_raw` | 图像发布话题名称。 | 启动时生效 |
| `exposure_time` | double | `10000.0` | 曝光时间，单位：微秒 (us)。需 $\ge 0$。 | 实时生效。若开启自动曝光，设置将被忽略。 |
| `gain` | double | `0.0` | 增益，单位：分贝 (dB)。需 $\ge 0$。 | 实时生效。若开启自动增益，设置将被忽略。 |
| `frame_rate` | double | `15.0` | 目标帧率，单位：Hz。需 $> 0$。 | 实时生效。实际帧率受曝光时间和带宽限制。 |
| `pixel_format` | string | `Mono8` | 像素格式 (如 `Mono8`, `RGB8Packed`, `BayerRG8`)。 | 建议启动时设定。动态切换可能导致短暂丢帧。 |
| `exposure_auto` | bool | `false` | 是否开启自动曝光。 | 实时生效。开启后手动曝光设置失效。 |
| `gain_auto` | bool | `false` | 是否开启自动增益。 | 实时生效。开启后手动增益设置失效。 |

**参数校验与反馈**：
- 设置负数曝光或增益时，节点将拒绝修改并返回明确错误原因（如 `Exposure time must be >= 0`）。
- 若 SDK 内部设置失败（如超出相机物理范围），节点将捕获错误码并返回 `SDK failed to set ...`。

## 5. 功能验证

### 5.1 查看图像 (RViz2)
1. 启动节点后，打开新终端运行 `rviz2`。
2. 将 `Fixed Frame` 设置为 `camera_optical_frame`。
3. 添加 `Image` 插件，订阅话题 `/image_raw`。
4. 预期结果：画面稳定显示相机采集的实时图像（若使用 `Mono8` 则为灰度图）。

### 5.2 查看帧率
```bash
ros2 topic hz /image_raw
```
*注意：区分“设置帧率”与“实际接收帧率”。若曝光时间过长（如 100,000us），实际帧率将远低于设置值。*

### 5.3 动态修改参数
在节点运行时，可通过命令行修改参数并观察图像变化：
```bash
# 增加曝光时间（图像变亮）
ros2 param set /hikrobot_camera_node exposure_time 20000.0

# 增加增益
ros2 param set /hikrobot_camera_node gain 10.0

# 尝试非法输入（应返回失败）
ros2 param set /hikrobot_camera_node exposure_time -100.0
```

## 6. 已知问题与优化建议 (WSL 环境)

在 WSL2 环境下通过 `usbipd` 映射 USB 相机时，由于虚拟总线带宽限制，高数据量传输可能触发 SDK 缓冲区溢出错误 (`0x80000007`)。

**优化方案**：
1. **推荐格式**：默认使用 `Mono8` 格式，其数据量仅为 `RGB8Packed` 的 1/3，可显著提升稳定性。
2. **降低帧率/分辨率**：若必须使用彩色格式，建议在代码中通过 ROI 裁剪降低分辨率，或将 `frame_rate` 降至 10Hz 以下。
3. **增大缓冲区**：代码中已默认将 SDK 接收缓冲区增大至 50MB (`GrabBufferSize`) 以缓解丢包。

## 7. 工程结构

```text
robomaster-camera-assignment/
├── config/
│   └── camera.yaml          # 默认参数配置
├── include/hikrobot_camera/
│   └── camera_node.hpp      # 节点头文件
├── launch/
│   └── camera.launch.py     # 启动文件
├── src/
│   ├── camera_node.cpp      # 核心节点实现
│   └── main.cpp             # 入口函数
├── CMakeLists.txt           # 编译配置
├── package.xml              # 依赖声明
└── README.md                # 本说明文档
```