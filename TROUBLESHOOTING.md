# XC-AGV 问题排查手册

本文档记录底盘系统调试过程中遇到的所有问题及解决方案。

---

## 目录

| # | 问题 | 关键词 |
|---|---|---|
| [1](#1-电机无响应sdo-命令字长度错误) | 电机无响应（SDO 命令字长度错误） | CANopen, U16, I8 |
| [2](#2-速度反馈异常socketcan-帧竞争) | 速度反馈异常（SocketCAN 帧竞争） | candump, 速度偏移 |
| [3](#3-编码器-position-方向不一致) | 编码器 position 方向不一致 | 里程计反向, 原地旋转 |
| [4](#4-速度单位转换错误01-rmin) | 速度单位转换错误（0.1 r/min） | velocity 异常, 速度偏大 |
| [5](#5-右轮速度方向错误) | 右轮速度方向错误 | 镜像安装, 转向偏 |
| [6](#6-里程计刻度不准确编码器校准) | 里程计刻度不准确（编码器校准） | counts/rev, 16957 |
| [7](#7-cmd_vel-话题命名空间问题) | cmd_vel 话题命名空间问题 | Nav2, remap |
| [8](#8-twist-vs-twiststamped-消息类型冲突) | Twist vs TwistStamped 消息类型冲突 | use_stamped_vel |
| [9](#9-雷达与-ros2_control-同时启动-lifecycle-冲突) | 雷达与 ros2_control 同时启动 lifecycle 冲突 | LifecycleNode, 电机无响应 |
| [10](#10-雷达点数波动导致-slam_toolbox-报错) | 雷达点数波动导致 slam_toolbox 报错 | binning, 836点 |
| [11](#11-系统重启后电机无响应需要初始化) | 系统重启后电机无响应（需要初始化） | 开机, Python脚本 |
| [12](#12-nav2-找不到-map-坐标系) | Nav2 找不到 map 坐标系 | AMCL, 初始位姿 |
| [13](#13-can-接口进入-error-passive--bus-off) | CAN 接口进入 ERROR-PASSIVE / Bus-Off | berr-counter, 重置 |
| [14](#14-编译内存不足-nav2_mppi_controller-崩溃) | 编译内存不足（nav2_mppi_controller 崩溃） | cc1plus killed, OOM |
| [15](#15-novnc-重启后失效) | noVNC 重启后失效 | VNC, 6080端口 |
| [16](#16-can-通信无信号驱动器拨盘配置) | CAN 通信无信号（驱动器拨盘配置） | 拨码开关, 节点ID |

---

## 1. 电机无响应（SDO 命令字长度错误）

### 现象
- `/cmd_vel` 话题可以正常发布和接收
- 控制器加载成功，无报错
- CAN 命令正常发送（candump 可见），但电机没有任何反应
- Python 脚本可以正常控制电机，但 ROS 程序不行

### 原因
ZLAC8015D 手册规定：
- `0x6040`（控制字 Controlword）：数据类型为 **U16（2 字节）**
- `0x6060`（运行模式）：数据类型为 **I8（1 字节）**

C++ 驱动错误地传入了 4 字节，导致 SDO 命令字为 `0x23`（写 4 字节），而非 `0x2B`（写 2 字节）/ `0x2F`（写 1 字节）。驱动器收到错误命令后拒绝使能，电机不响应。

### 解决方案
在 `zlac8015d_can_driver.cpp` 的 `activateVelocityMode()` 中，严格按数据类型传入正确长度：

```cpp
uint8_t ctrl_data[2] = {0x06, 0x00};
sendSDO(ADDR_CONTROLWORD, 0x00, ctrl_data, 2);  // U16 → len=2

uint8_t mode_data[1] = {0x03};
sendSDO(ADDR_OPERATION_MODE, 0x00, mode_data, 1);  // I8 → len=1
```

---

## 2. 速度反馈异常（SocketCAN 帧竞争）

### 现象
静止时 velocity 间歇性显示异常大值（如 -257.61 或 716.07 rad/s），而非 0。

### 原因
`readSDO()` 函数读取响应帧时，可能读到 CAN 总线上的其他无关帧。原代码只验证了 CAN ID，未验证帧内容（索引/子索引）。

### 解决方案
在 `readSDO()` 中添加完整帧验证，循环读取直到索引和子索引完全匹配：

```cpp
bool index_match = (frame_rx.data[1] == (index & 0xFF)) &&
                   (frame_rx.data[2] == ((index >> 8) & 0xFF));
bool subindex_match = (frame_rx.data[3] == subindex);
if (!index_match || !subindex_match) continue;
```

同时在 `init()` 中将 socket 设为非阻塞模式，`readSDO()` 使用 `select()` 实现 5ms 超时，防止阻塞控制循环。

---

## 3. 编码器 position 方向不一致

### 现象
小车向前推动时，左轮 position 增加，右轮 position 减少（方向相反）。RViz 中显示机器人原地旋转而非直线前进。

### 原因
ZLAC8015D 双电机控制器读取的编码器方向与 URDF 定义不一致（右轮镜像安装）。

### 解决方案
在驱动代码中对左右轮 position 均取反：

```cpp
double radians = (counts / 16957.0) * 2.0 * M_PI;
return -radians;  // 左右轮均取反
```

---

## 4. 速度单位转换错误（0.1 r/min）

### 现象
速度反馈数值与实际运动速度不匹配（偏大约 10 倍）。

### 原因
ZLAC8015D 手册规定 `0x606C`（Actual Velocity）单位为 **0.1 r/min**，而非 r/min。原代码直接当 RPM 处理。

### 解决方案

```cpp
int32_t raw_value = bytesToInt32(data);  // 单位：0.1 r/min
double rpm = raw_value * 0.1;            // → r/min
double vel_rad_s = rpm * RPM_TO_RAD_S;  // → rad/s
```

---

## 5. 右轮速度方向错误

### 现象
速度单位修正后，发现小车转向偏斜，右轮速度方向需要校正。

### 原因
右轮镜像安装，速度反馈方向相反。

### 解决方案
在右轮速度读取函数中取反：

```cpp
return -vel_rad_s;  // 右轮取反
```

---

## 6. 里程计刻度不准确（编码器校准）

### 现象
手动转动轮子一圈，RViz 中 position 变化约 52 弧度，而非理论值 2π ≈ 6.28 弧度。

### 原因
编码器安装在电机轴（非轮轴），包含减速比效果，原始计算未考虑减速比。

### 解决方案
实测校准：轮子转一圈的实际 counts 数约为 16957（编码器分辨率 × 倍频系数 × 减速比综合值）：

```cpp
double radians = (counts / 16957.0) * 2.0 * M_PI;
```

> **校准方法**：手动转轮子整整一圈，记录 position 变化值，计算 `2048 × (实测值 / 2π)` 得到新校准系数。

---

## 7. cmd_vel 话题命名空间问题

### 现象
Nav2 和 teleop 发布到 `/cmd_vel`，但控制器订阅 `/diff_drive_controller/cmd_vel`，互不通信。

### 原因
diff_drive_controller 默认在话题名前加控制器名称作为命名空间。remap 必须加在 `ros2_control_node` 上（而非 `spawner`），因为控制器实际运行在 `ros2_control_node` 进程中。

### 解决方案
在 `xiaoche_hardware.launch.xml` 的 `ros2_control_node` 节点上添加 remap：

```xml
<node pkg="controller_manager" exec="ros2_control_node">
    ...
    <remap from="/diff_drive_controller/cmd_vel_unstamped" to="/cmd_vel" />
</node>
```

---

## 8. Twist vs TwistStamped 消息类型冲突

### 现象
发布 `/cmd_vel` 后控制器无响应，`ros2 topic echo /cmd_vel` 报两种类型冲突。

### 原因
diff_drive_controller 默认使用 TwistStamped（`use_stamped_vel: true`），话题名为 `cmd_vel`；但 Nav2/teleop 默认发布 Twist，话题名为 `cmd_vel`。类型不匹配，无法通信。

### 解决方案
在 `xiaoche_controllers.yaml` 中设置：

```yaml
diff_drive_controller:
  ros__parameters:
    use_stamped_vel: false  # 使用 Twist，话题变为 cmd_vel_unstamped
```

同步更新 launch 文件中的 remap（`cmd_vel` → `cmd_vel_unstamped`）。

---

## 9. 雷达与 ros2_control 同时启动 lifecycle 冲突

### 现象
在同一 launch 文件中同时启动雷达（lslidar_driver，LifecycleNode）和 ros2_control，所有节点加载成功无报错，但电机完全无响应。

### 原因
ros2_control 内部使用 lifecycle 管理硬件组件。lslidar_driver 也是 LifecycleNode，两者同时启动时 lifecycle 状态转换相互干扰，导致硬件接口未真正激活。（参考 [ros2_control Issue #2079](https://github.com/ros-controls/ros2_control/issues/2079)）

### 解决方案
**分开启动**（当前采用，最可靠）：

```bash
# 终端1：先启动底盘，等待控制器完全加载（约 3 秒）
ros2 launch my_robot_bringup xiaoche_hardware.launch.xml

# 终端2：再启动雷达
ros2 launch lslidar_driver lsm10p_uart_launch.py
```

---

## 10. 雷达点数波动导致 slam_toolbox 报错

### 现象
slam_toolbox 持续报 `LaserRangeScan contains 837 range readings, expected 836`，拒绝扫描数据。

### 原因
M10P 雷达电机转速不稳定，每圈输出点数在 831-837 之间波动。slam_toolbox 基于第一帧固定期望点数，后续不匹配则丢弃。

### 解决方案
在雷达驱动和 slam_toolbox 之间插入 Python binning 节点，将 `/scan_raw` 固定重采样为 836 点后发布到 `/scan`：

- 雷达驱动发布 `/scan_raw`（点数波动）
- binning 节点订阅 `/scan_raw`，发布固定 836 点的 `/scan`
- binning 节点延迟 3 秒启动（避免与雷达驱动初始化冲突）

关键参数：`angle_increment = total_range / (num_bins - 1)`（适配 slam_toolbox 期望值计算公式）。

---

## 11. 系统重启后电机无响应（需要初始化）

### 现象
每次系统重启后，直接运行 ROS2 控制程序，电机完全无响应。必须先运行 Python 键盘控制程序后，ROS2 程序才能正常工作。

### 原因
ROS2 驱动的 `activateVelocityMode()` 初始化步骤不完整，缺少 Python 程序中的若干步骤（关闭心跳、设置异步控制模式、配置加速时间等），导致电机控制器未正确进入可响应状态。

### 解决方案（临时）
每次开机后，启动主程序前先运行：

```bash
python3 ~/keyboard_can_control/keyboard_can_control_linux.py
# 启动后按 X 或 ESC 退出
```

---

## 12. Nav2 找不到 map 坐标系

### 现象
Nav2 启动后报错 `Timed out waiting for transform from base_link to map`，`frame "map" does not exist`。

### 原因
AMCL 负责发布 `map→odom` TF 变换，但 AMCL 需要先收到初始位姿才会开始发布。未设置初始位姿时，map 坐标系不存在。

### 解决方案
在 `nav2_params.yaml` 的 `amcl` 部分添加自动初始位姿：

```yaml
amcl:
  ros__parameters:
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
```

或在 rviz 中手动点击「2D Pose Estimate」设置初始位姿。

> rviz 中 Map 显示需将 Durability Policy 设为 **Transient Local**。

---

## 13. CAN 接口进入 ERROR-PASSIVE / Bus-Off

### 现象
`ip -details link show can0` 显示 `can state ERROR-PASSIVE` 或 `Bus-Off`，CAN 总线无通信。

### 原因
- 电机控制器未通电（最常见）
- CAN H/L 接线接反
- GND 未共地
- 缺少终端电阻（两端各 120Ω）

### 排查步骤

```bash
# 1. 查看详细错误计数
ip -details link show can0

# 2. 回环测试验证板子 CAN 硬件
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000 loopback on
candump can0 &
cansend can0 601#2B.1D.20.01.2C.01.00.00
# 应看到自己发的帧被接收
```

### 恢复命令

```bash
sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 500000 restart-ms 100
```

---

## 14. 编译内存不足（nav2_mppi_controller 崩溃）

### 现象
编译报错 `c++: fatal error: 已杀死 signal terminated program cc1plus`，nav2_mppi_controller 编译失败。

### 原因
ARM 设备内存有限，nav2_mppi_controller 是模板密集型 C++ 代码，多线程并行编译时内存峰值超出限制。

### 解决方案
单线程编译：

```bash
export MAKEFLAGS="-j 1"
colcon build --packages-select nav2_mppi_controller --allow-overriding nav2_mppi_controller
```

---

## 15. noVNC 重启后失效

### 现象
RDK-X5 重启后，浏览器无法访问 `http://<IP>:6080/vnc.html`。

### 原因
noVNC 服务开机启动时，x11vnc 尚未完全就绪，导致 noVNC 启动失败后不再重试。

### 解决方案
在 `/etc/systemd/system/novnc.service` 的 `[Service]` 段添加启动延迟：

```ini
[Service]
ExecStartPre=/bin/sleep 5
...
Restart=always
RestartSec=5
```

手动恢复：

```bash
sudo systemctl restart novnc
```

---

## 16. CAN 通信无信号（驱动器拨盘配置）

### 现象
CAN 接线正确，板子 CAN 硬件正常（回环测试通过），但接上驱动器后 candump 无任何响应帧（581）。

### 原因
ZLAC8015D 驱动器通过拨码开关设置 CAN 节点 ID。出厂默认两个开关均朝上（节点 ID 不正确），无法建立通信。

### 解决方案
调整驱动器侧拨码开关：**1 朝下，2 朝上**（对应节点 ID = 1）。重新上电后恢复通信。
