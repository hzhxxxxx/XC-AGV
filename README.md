# XC-AGV · ROS2 差速底盘系统

基于 RDK-X5 + ROS2 Humble 的差速驱动 AGV 底盘，使用 ZLAC8015D CAN 总线电机控制器，支持激光雷达 SLAM 建图、Nav2 自主导航及 HTTP/WebSocket 上位机通信。

---

## 硬件配置

| 组件 | 型号 | 说明 |
| --- | --- | --- |
| 主控板 | RDK-X5 (ARM64) | ROS2 Humble |
| 电机控制器 | ZLAC8015D V4.0 | CAN 总线，CANopen 协议，节点 ID=1 |
| 激光雷达 | 镭神 M10P / M10（串口） | `/dev/ttyACM0`，460800 波特率 |
| IMU | WIT 系列 | `/dev/ttyUSB0` → udev 别名 `/dev/imu_usb` |
| 深度相机 | 海康 MV-EB435i（可选） | USB，需单独安装 SDK |

**底盘参数：**
- 轮间距：0.36 m
- 轮半径：0.0903 m
- 编码器校准系数：16957 counts/rev（含减速比）
- CAN 波特率：500 kHz

---

## 移植安装（新板迁移指南）

### 1. 前置条件

新板需已安装 ROS2 Humble，且用户在 `dialout` 组（`groups` 命令验证）。

### 2. 复制源码

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b 2.0 https://github.com/hzhxxxxx/XC-AGV.git .
```

### 3. 安装依赖

```bash
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-robot-localization
sudo apt install -y ros-humble-rviz2 ros-humble-xacro ros-humble-teleop-twist-keyboard
sudo apt install -y libxtensor-dev libxsimd-dev python3-can python3-pynput

# RTAB-Map 建图
sudo apt install -y ros-humble-rtabmap-ros
pip install rtabmap-python
```

### 4. 编译工作空间

`nav2_mppi_controller` 在 ARM64 Humble 二进制包有 SIGILL 崩溃问题，必须从源码编译：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# 先编译其他包（跳过海康相机，SDK 需单独安装）
colcon build --packages-ignore mv3d_rgbd_ros2 mv3d_rgbd_ros2_interface nav2_mppi_controller

# 单线程编译 mppi（-j1 防止内存不足）
export MAKEFLAGS="-j 1"
colcon build --packages-select nav2_mppi_controller --allow-overriding nav2_mppi_controller

source install/setup.bash
```

### 5. 配置系统环境

**`.bashrc` 环境变量：**

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**CAN 接口开机自启（`can0.service`）：**

```bash
sudo bash -c 'cat > /etc/systemd/system/can0.service << EOF
[Unit]
Description=CAN Interface Setup
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/ip link set can0 down
ExecStart=/sbin/ip link set can0 up type can bitrate 500000 restart-ms 100
ExecStop=/sbin/ip link set can0 down
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF'
sudo systemctl daemon-reload && sudo systemctl enable can0.service && sudo systemctl start can0.service
```

**IMU udev 规则：**

```bash
sudo bash -c 'echo "KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", MODE:=\"0777\", SYMLINK+=\"imu_usb\"" > /etc/udev/rules.d/imu_usb.rules'
sudo udevadm control --reload-rules && sudo udevadm trigger
# 拔插 IMU USB 使规则生效
```

**雷达 udev 规则（固定串口别名）：**

两个雷达 USB 芯片相同（1a86:55d4），必须通过序列号区分，否则插入顺序变化会导致串口编号漂移。

M10 规则（`/etc/udev/rules.d/lslidar_m10.rules`）：

```bash
sudo bash -c 'echo "KERNEL==\"ttyACM*\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"55d4\", ATTRS{serial}==\"5A6D014086\", SYMLINK+=\"lslidar_m10\", MODE:=\"0666\"" > /etc/udev/rules.d/lslidar_m10.rules'
```

**MS200 规则（`/etc/udev/rules.d/ms200.rules`）：**

```bash
sudo bash -c 'echo "KERNEL==\"ttyACM*\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"55d4\", ATTRS{serial}==\"597D000635\", SYMLINK+=\"lslidar_ms200\", MODE:=\"0666\"" > /etc/udev/rules.d/ms200.rules'
```

重载规则并拔插两个雷达 USB 使别名生效：

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

效果：`/dev/lslidar_m10`、`/dev/lslidar_ms200` 固定绑定对应雷达，与插入顺序无关。

**复制地图文件：**

```bash
mkdir -p ~/maps
# 将 xc_room1.pgm / xc_room1.yaml 复制到 ~/maps/
```

---

## 启动命令

> ⚠️ **每次开机后，建议先完成以下 CAN 通信验证步骤，确认电机控制正常后再启动主程序。**

```bash
# 0a. 激活 CAN 接口（如已配置 can0.service 开机自启则跳过）
sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 500000

# 0b. 手动电机测试（推荐）：验证 CAN 通信及电机响应是否正常
#     运行后可用键盘前后左右控制电机，按 X 或 ESC 退出
python3 /home/sunrise/keyboard_can_control/keyboard_can_control_linux.py
```
此步骤非必须，但建议执行以排除 CAN 总线通信异常，避免后续启动后电机无响应。

按顺序在各终端启动：

```bash
# 1. 底盘主程序
ros2 launch my_robot_bringup xiaoche_hardware.launch.xml

# 2. IMU
ros2 run wit_ros2_imu wit_ros2_imu

# 3. EKF 融合
ros2 launch my_robot_bringup ekf.launch.py

# 4. 激光雷达
-前雷达启动：
  rdk-x5-1 : `ros2 launch lslidar_driver lsm10p_uart_launch.py`
  rdk-x5-2 : `ros2 launch lslidar_driver lsm10_uart_launch.py`

-后雷达启动 : `ros2 launch oradar_lidar ms200_scan.launch.py`

-启动双雷达融合 : `ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py`

# 5. Nav2 导航
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  map:=/home/sunrise/maps/xc_room1.yaml \
  params_file:=/home/sunrise/ros2_ws/src/my_robot_bringup/config/nav2_params.yaml

# 6. 海康相机启动（可选）：
`ros2 run mv3d_rgbd_ros2 hik_camera_image_pub`

# 6. 上位机通信服务
cd ~/ros2_ws && source install/setup.bash

-智能小R：
python3 src/my_robot_bringup/scripts/xc_robot_server.py

-Autonomous：
python3 src/my_robot_bringup/scripts/robot_claw_server.py

# 可选：SLAM 建图
- `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false`
- `ros2 launch slam_toolbox online_async_launch.py params_file:=$HOME/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/config/slam_params.yaml`
- `ros2 launch my_robot_bringup rtabmap.launch.py`
- `ros2 run nav2_map_server map_saver_cli -f maps/xc_room2`

# 可选：键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 上位机通信 API

系统提供两套 HTTP REST（端口 8080）+ WebSocket 推送（端口 8081）的上位机通信程序：

| 程序 | 文件 | 端口 IP | 适用场景 |
| --- | --- | --- | --- |
| **Autonomous Robot** | `robot_claw_server.py` | `10.10.92.174:8080` / `10.10.91.86:8080` | 通用底盘控制，支持持续手动控制、相机抓拍 |
| **智能小R** | `xc_robot_server.py` | `10.10.91.86:8080` | 简易运动模式，支持指定米数/角度一次性运动 |

### 通用命令

以下接口在两个 server 中均可用（以 `10.10.91.86` 为例）：

```bash
# 导航到点位
curl -s -X POST http://10.10.91.86:8080/api/v1/navigate \
  -H "Content-Type: application/json" \
  -d '{"point_id": "home"}' | python3 -m json.tool

# 立即停车
curl -s -X POST http://10.10.91.86:8080/api/v1/stop \
  -H "Content-Type: application/json" \
  -d '{"reason": "user_command"}' | python3 -m json.tool

# 查询状态
curl -s http://10.10.91.86:8080/api/v1/status | python3 -m json.tool

# WebSocket 推送
# 连接地址：ws://<robot-ip>:8081/ws，导航状态变化时自动推送
```

### 手动控制（Autonomous Robot）

持续运动模式，需周期性发送保持运动，内置 2 秒 Watchdog 超时自动停车。

```bash
# 持续前进（speed_level: slow / normal / fast）
curl -s -X POST http://10.10.92.174:8080/api/v1/move/forward \
  -H "Content-Type: application/json" \
  -d '{"speed_level": "normal"}' | python3 -m json.tool

# 持续后退 / 左转 / 右转（用法同上，端点分别为 backward / left / right）

# 导航到坐标（无需预设点位）
curl -s -X POST http://10.10.92.174:8080/api/v1/navigate/pose \
  -H "Content-Type: application/json" \
  -d '{"x": 0.88, "y": -0.51, "theta": -1.59}' | python3 -m json.tool

# 查询点位列表
curl -s http://10.10.92.174:8080/api/v1/points | python3 -m json.tool
```

> **Watchdog 说明**：每次收到 `/move/*` 指令后 2 秒未收到新指令则自动停车。上层应以 200~500ms 间隔连续发送指令维持运动。

### 手动控制（智能小 R）

一次性运动模式，指定米数/角度后自动到位停车。

```bash
# 前进指定米数
curl -s -X POST http://10.10.91.86:8080/api/v1/move/forward \
  -H "Content-Type: application/json" \
  -d '{"amount": 0.3}' | python3 -m json.tool

# 后退指定米数
curl -s -X POST http://10.10.91.86:8080/api/v1/move/backward \
  -H "Content-Type: application/json" \
  -d '{"amount": 0.3}' | python3 -m json.tool

# 旋转到绝对角度（度）
curl -s -X POST http://10.10.91.86:8080/api/v1/move/rotate \
  -H "Content-Type: application/json" \
  -d '{"amount": 90}' | python3 -m json.tool
```

### 相机 API（仅 Autonomous Robot，需先启动相机节点）

```bash
# 抓拍
curl -s -X POST http://10.10.91.86:8080/api/v1/camera/capture | python3 -m json.tool

# 抓拍（含深度图）
curl -s -X POST http://10.10.91.86:8080/api/v1/camera/capture \
  -H "Content-Type: application/json" \
  -d '{"include_depth": true}' | python3 -m json.tool

# 下载图片（image_id 从抓拍返回中获取）
curl -o rgb.jpg http://10.10.91.86:8080/api/v1/camera/images/<image_id>/rgb
curl -o depth.png http://10.10.91.86:8080/api/v1/camera/images/<image_id>/depth
```

### 点位配置

编辑 `my_robot_bringup/config/poi_map.json`，无需改代码。

可用点位及坐标详见 `poi_map.json`，支持 `home` / `point_1` ~ `point_7`。

---

## VNC 远程

1. SSH 连上目标机器后，重启 noVNC 服务：

```bash
sudo systemctl restart novnc
```

1. 浏览器访问：
   - **rdk-x5**：`http://10.10.92.174:6080/vnc.html`，密码 `sunrise`
   - **rdk-x5-2**：`http://10.10.91.86:6080/vnc.html`，密码 `sunrise`

---

## 问题排查

调试过程中遇到的所有问题及解决方案，见 **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)**。
