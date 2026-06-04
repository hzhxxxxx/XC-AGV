# XC-AGV · ROS2 差速底盘系统

基于 RDK-X5 + ROS2 Humble 的差速驱动 AGV 底盘，使用 ZLAC8015D CAN 总线电机控制器，支持激光雷达 SLAM 建图、Nav2 自主导航及 HTTP/WebSocket 上位机通信。

---

## 硬件配置

| 组件 | 型号 | 说明 |
|---|---|---|
| 主控板 | RDK-X5 (ARM64) | ROS2 Humble |
| 电机控制器 | ZLAC8015D V4.0 | CAN 总线，CANopen 协议，节点 ID=1 |
| 激光雷达 | 镭神 M10P / M10（串口） | `/dev/ttyACM0`，460800 波特率 |
| IMU | WIT 系列 | `/dev/ttyUSB0` → udev 别名 `/dev/imu_usb` |
| 深度相机 | 海康 MV-EB435i（可选） | USB，需单独安装 SDK |

**底盘参数：**
- 轮间距：0.36 m
- 轮半径：0.0825 m
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

**雷达 udev 规则（固定串口别名）：**

两个雷达 USB 芯片相同（1a86:55d4），必须通过序列号区分，否则插入顺序变化会导致串口编号漂移。

M10 规则（`/etc/udev/rules.d/lslidar_m10.rules`）：

```bash
sudo bash -c 'echo "KERNEL==\"ttyACM*\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"55d4\", ATTRS{serial}==\"5A6D014086\", SYMLINK+=\"lslidar_m10\", MODE:=\"0666\"" > /etc/udev/rules.d/lslidar_m10.rules'
```

MS200 规则（`/etc/udev/rules.d/ms200.rules`）：

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

> ⚠️ **每次开机后，首次启动主程序前必须先运行电机初始化脚本，否则电机无响应。**

```bash
# 0. 电机初始化（首次启动前执行，按 X 或 ESC 退出）
python3 ~/keyboard_can_control/keyboard_can_control_linux.py
```

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

# 6. 海康相机启动：
`ros2 run mv3d_rgbd_ros2 hik_camera_image_pub`

# 6. 上位机通信服务
cd ~/ros2_ws && source install/setup.bash

-智能小R：
python3 src/my_robot_bringup/scripts/xc_robot_server.py

-Autonomous:
python3 src/my_robot_bringup/scripts/robot_claw_server.py

# 可选：SLAM 建图
- `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false`
- `ros2 launch slam_toolbox online_async_launch.py params_file:=$HOME/ros2_ws/install/my_robot_bringup/share/my_robot_bringup/config/slam_params.yaml`
- `ros2 run nav2_map_server map_saver_cli -f maps/xc_room2`

# 可选：键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 上位机通信 API

HTTP REST（端口 8080）+ WebSocket 推送（端口 8081）。

### 导航到目标点

```bash
curl -X POST http://<X5_IP>:8080/api/v1/navigate \
  -H "Content-Type: application/json" \
  -d '{"point_id": "work"}'
```

### 立即停车

```bash
curl -X POST http://<X5_IP>:8080/api/v1/stop \
  -H "Content-Type: application/json" \
  -d '{"reason": "user_command"}'
```

### 查询状态

```bash
curl http://<X5_IP>:8080/api/v1/status
```

`nav.state` 取值：`idle` / `navigating` / `arrived` / `failed`

### WebSocket 推送

连接地址：`ws://<X5_IP>:8081/ws`，状态变化时自动推送。

### 点位配置

编辑 `my_robot_bringup/config/poi_map.json`，无需改代码。

---

## 遥控访问

| 方式 | 地址 | 密码 |
|---|---|---|
| noVNC（浏览器） | `http://<X5_IP>:6080/vnc.html` | `sunrise` |

---

## 问题排查

调试过程中遇到的所有问题及解决方案，见 **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)**。
