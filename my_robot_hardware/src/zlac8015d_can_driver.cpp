#include "my_robot_hardware/zlac8015d_can_driver.hpp"
#include <fcntl.h>  // 添加这个头文件用于fcntl
#include <sys/select.h>  // 添加select支持

ZLAC8015DCanDriver::ZLAC8015DCanDriver(const std::string& can_interface, int node_id)
    : can_interface_(can_interface),
      node_id_(node_id),
      can_socket_(-1),
      sdo_tx_(0x600 + node_id),
      sdo_rx_(0x580 + node_id),
      left_position_(0.0),
      right_position_(0.0),
      last_left_velocity_(0.0),
      last_right_velocity_(0.0),
      last_left_position_(0.0),
      last_right_position_(0.0),
      left_position_initialized_(false),
      right_position_initialized_(false)
{
    std::cout << "[ZLAC8015D] 初始化驱动: " << can_interface << ", 节点ID=" << node_id << std::endl;
}

ZLAC8015DCanDriver::~ZLAC8015DCanDriver()
{
    if (can_socket_ >= 0) {
        close(can_socket_);
        std::cout << "[ZLAC8015D] CAN接口已关闭" << std::endl;
    }
}

int ZLAC8015DCanDriver::init()
{
    std::cout << "[ZLAC8015D] 正在打开CAN接口: " << can_interface_ << std::endl;

    // 创建SocketCAN套接字
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        std::cerr << "[ZLAC8015D] 创建套接字失败!" << std::endl;
        return -1;
    }

    // 获取CAN接口索引
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "[ZLAC8015D] 获取接口索引失败! 请检查CAN接口是否启动" << std::endl;
        std::cerr << "  提示: sudo ip link set " << can_interface_ << " type can bitrate 500000" << std::endl;
        std::cerr << "        sudo ip link set " << can_interface_ << " up" << std::endl;
        close(can_socket_);
        can_socket_ = -1;
        return -1;
    }

    // 绑定CAN接口
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "[ZLAC8015D] 绑定接口失败!" << std::endl;
        close(can_socket_);
        can_socket_ = -1;
        return -1;
    }

    // *** 方案A关键修改：设置socket为非阻塞模式 ***
    int flags = fcntl(can_socket_, F_GETFL, 0);
    if (flags == -1) {
        std::cerr << "[ZLAC8015D] 获取socket flags失败!" << std::endl;
        close(can_socket_);
        can_socket_ = -1;
        return -1;
    }
    if (fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK) == -1) {
        std::cerr << "[ZLAC8015D] 设置非阻塞模式失败!" << std::endl;
        close(can_socket_);
        can_socket_ = -1;
        return -1;
    }

    std::cout << "[ZLAC8015D] CAN接口打开成功（非阻塞模式）" << std::endl;

    // 开始配置电机参数
    std::cout << "[ZLAC8015D] 开始配置电机参数..." << std::endl;

    // 1. 关闭心跳 (0x1017)
    uint8_t heartbeat_data[4] = {0x00, 0x00, 0x00, 0x00};
    if (!sendSDO(ADDR_HEARTBEAT, 0x00, heartbeat_data, 4)) {
        std::cerr << "[ZLAC8015D] 关闭心跳失败" << std::endl;
        return -1;
    }

    // 2. 设置异步控制模式 (0x200F)
    uint8_t async_data[4] = {0x00, 0x00, 0x00, 0x00};
    if (!sendSDO(ADDR_ASYNC_CONTROL, 0x00, async_data, 4)) {
        std::cerr << "[ZLAC8015D] 设置异步控制失败" << std::endl;
        return -1;
    }

    // 3. 配置S曲线加速时间 (0x6083)
    std::cout << "[ZLAC8015D] 配置加速参数: 加速=1000ms, 减速=800ms" << std::endl;
    setAccelerationTime(1000);
    setDecelerationTime(800);

    // 4. 配置急停时间 (0x6085) - Python脚本中有
    std::cout << "[ZLAC8015D] 配置急停时间=100ms" << std::endl;
    uint8_t quick_stop_bytes[4];
    uint32ToBytes(100, quick_stop_bytes);  // 100ms急停
    if (!sendSDO(ADDR_QUICK_STOP_TIME, SUBINDEX_LEFT_MOTOR, quick_stop_bytes, 4)) {
        std::cerr << "[ZLAC8015D] 设置左电机急停时间失败" << std::endl;
        return -1;
    }
    if (!sendSDO(ADDR_QUICK_STOP_TIME, SUBINDEX_RIGHT_MOTOR, quick_stop_bytes, 4)) {
        std::cerr << "[ZLAC8015D] 设置右电机急停时间失败" << std::endl;
        return -1;
    }

    // 5. 配置转矩斜率 (0x6087)
    std::cout << "[ZLAC8015D] 配置转矩斜率=500mA/s" << std::endl;
    setTorqueSlope(500);

    // 6. 配置最大电流 (0x2015) - 30A
    std::cout << "[ZLAC8015D] 配置最大电流=30A" << std::endl;
    uint8_t max_current_bytes[4];
    uint32ToBytes(300, max_current_bytes);  // 300 * 0.1A = 30A
    if (!sendSDO(ADDR_MAX_CURRENT, SUBINDEX_LEFT_MOTOR, max_current_bytes, 4)) {
        std::cerr << "[ZLAC8015D] 设置左电机最大电流失败" << std::endl;
        return -1;
    }
    if (!sendSDO(ADDR_MAX_CURRENT, SUBINDEX_RIGHT_MOTOR, max_current_bytes, 4)) {
        std::cerr << "[ZLAC8015D] 设置右电机最大电流失败" << std::endl;
        return -1;
    }

    // 7. 配置过载系数 (0x2012) - 250%
    std::cout << "[ZLAC8015D] 配置过载系数=250%" << std::endl;
    uint8_t overload_ratio_bytes[4];
    uint32ToBytes(250, overload_ratio_bytes);  // 250%
    if (!sendSDO(ADDR_OVERLOAD_RATIO, SUBINDEX_LEFT_MOTOR, overload_ratio_bytes, 4)) {
        std::cerr << "[ZLAC8015D] 设置左电机过载系数失败" << std::endl;
        return -1;
    }
    if (!sendSDO(ADDR_OVERLOAD_RATIO, SUBINDEX_RIGHT_MOTOR, overload_ratio_bytes, 4)) {
        std::cerr << "[ZLAC8015D] 设置右电机过载系数失败" << std::endl;
        return -1;
    }

    // 8. 配置过载保护时间 (0x2016) - 5秒
    std::cout << "[ZLAC8015D] 配置过载保护时间=5秒" << std::endl;
    uint8_t overload_time_bytes[4];
    uint32ToBytes(500, overload_time_bytes);  // 500 * 10ms = 5秒
    if (!sendSDO(ADDR_OVERLOAD_TIME, SUBINDEX_LEFT_MOTOR, overload_time_bytes, 4)) {
        std::cerr << "[ZLAC8015D] 设置左电机过载保护时间失败" << std::endl;
        return -1;
    }
    if (!sendSDO(ADDR_OVERLOAD_TIME, SUBINDEX_RIGHT_MOTOR, overload_time_bytes, 4)) {
        std::cerr << "[ZLAC8015D] 设置右电机过载保护时间失败" << std::endl;
        return -1;
    }

    std::cout << "[ZLAC8015D] 电机参数配置完成!" << std::endl;
    return 0;
}

void ZLAC8015DCanDriver::activateVelocityMode()
{
    std::cout << "[ZLAC8015D] 激活速度模式..." << std::endl;

    // *** 关键修复：0x6040是U16（2字节），0x6060是I8（1字节） ***

    // 1. 使能电机（状态机）- 0x6040控制字是U16类型，需要用2字节
    uint8_t ctrl_data[2];  // 修改为2字节

    // Shutdown (0x6040 = 0x06)
    std::cout << "[ZLAC8015D] 步骤1: Shutdown" << std::endl;
    ctrl_data[0] = 0x06; ctrl_data[1] = 0x00;
    if (!sendSDO(ADDR_CONTROLWORD, 0x00, ctrl_data, 2)) {  // len=2，使用0x2B命令
        std::cerr << "[ZLAC8015D] 使能步骤1失败" << std::endl;
        return;
    }
    usleep(50000);

    // Switch On (0x6040 = 0x07)
    std::cout << "[ZLAC8015D] 步骤2: Switch On" << std::endl;
    ctrl_data[0] = 0x07; ctrl_data[1] = 0x00;
    if (!sendSDO(ADDR_CONTROLWORD, 0x00, ctrl_data, 2)) {  // len=2，使用0x2B命令
        std::cerr << "[ZLAC8015D] 使能步骤2失败" << std::endl;
        return;
    }
    usleep(50000);

    // Enable Operation (0x6040 = 0x0F)
    std::cout << "[ZLAC8015D] 步骤3: Enable Operation" << std::endl;
    ctrl_data[0] = 0x0F; ctrl_data[1] = 0x00;
    if (!sendSDO(ADDR_CONTROLWORD, 0x00, ctrl_data, 2)) {  // len=2，使用0x2B命令
        std::cerr << "[ZLAC8015D] 使能步骤3失败" << std::endl;
        return;
    }
    usleep(50000);

    // 2. 设置速度模式 (0x6060 = 0x03) - 0x6060是I8类型，需要用1字节
    std::cout << "[ZLAC8015D] 设置速度模式" << std::endl;
    uint8_t mode_data[1] = {0x03};  // 修改为1字节
    if (!sendSDO(ADDR_OPERATION_MODE, 0x00, mode_data, 1)) {  // len=1，使用0x2F命令
        std::cerr << "[ZLAC8015D] 设置速度模式失败" << std::endl;
        return;
    }

    std::cout << "[ZLAC8015D] 速度模式激活成功!" << std::endl;
}

void ZLAC8015DCanDriver::deactivate()
{
    std::cout << "[ZLAC8015D] 停用电机" << std::endl;
    setTargetVelocityRadianPerSec(0.0, 0.0);
    usleep(100000);

    // 0x6040控制字是U16（2字节）
    uint8_t ctrl_data[2] = {0x00, 0x00};
    sendSDO(ADDR_CONTROLWORD, 0x00, ctrl_data, 2);
}

void ZLAC8015DCanDriver::emergencyStop()
{
    std::cout << "[ZLAC8015D] 紧急停止!" << std::endl;
    setTargetVelocityRadianPerSec(0.0, 0.0);
}

void ZLAC8015DCanDriver::setAccelerationTime(uint32_t time_ms)
{
    uint8_t accel_bytes[4];
    uint32ToBytes(time_ms, accel_bytes);

    sendSDO(ADDR_ACCEL_TIME, SUBINDEX_LEFT_MOTOR, accel_bytes, 4);
    sendSDO(ADDR_ACCEL_TIME, SUBINDEX_RIGHT_MOTOR, accel_bytes, 4);
}

void ZLAC8015DCanDriver::setDecelerationTime(uint32_t time_ms)
{
    uint8_t decel_bytes[4];
    uint32ToBytes(time_ms, decel_bytes);

    sendSDO(ADDR_DECEL_TIME, SUBINDEX_LEFT_MOTOR, decel_bytes, 4);
    sendSDO(ADDR_DECEL_TIME, SUBINDEX_RIGHT_MOTOR, decel_bytes, 4);
}

void ZLAC8015DCanDriver::setTorqueSlope(uint32_t slope_ma_s)
{
    uint8_t slope_bytes[4];
    uint32ToBytes(slope_ma_s, slope_bytes);

    sendSDO(ADDR_TORQUE_SLOPE, SUBINDEX_LEFT_MOTOR, slope_bytes, 4);
    sendSDO(ADDR_TORQUE_SLOPE, SUBINDEX_RIGHT_MOTOR, slope_bytes, 4);
}

void ZLAC8015DCanDriver::setTargetVelocityRadianPerSec(double left_vel, double right_vel)
{
        // 右轮方向校正（根据机械安装，镜像对称）
        double actual_right_vel = -right_vel;  // 右轮取反

        // 转换: rad/s → RPM
        int32_t left_rpm = static_cast<int32_t>(left_vel * RAD_S_TO_RPM);
        int32_t right_rpm = static_cast<int32_t>(actual_right_vel * RAD_S_TO_RPM);

        uint8_t left_bytes[4];
        uint8_t right_bytes[4];
        int32ToBytes(left_rpm, left_bytes);
        int32ToBytes(right_rpm, right_bytes);

        sendSDO(ADDR_TARGET_VELOCITY, SUBINDEX_LEFT_MOTOR, left_bytes, 4);
        sendSDO(ADDR_TARGET_VELOCITY, SUBINDEX_RIGHT_MOTOR, right_bytes, 4);
}

double ZLAC8015DCanDriver::getLeftVelocityRadianPerSec()
{
    uint8_t data[4];
    if (readSDO(ADDR_ACTUAL_VELOCITY, SUBINDEX_LEFT_MOTOR, data, 4)) {
        int32_t raw_value = bytesToInt32(data);  // 单位：0.1 r/min
        double rpm = raw_value * 0.1;  // 转换为 r/min
        double vel_rad_s = rpm * RPM_TO_RAD_S;
        last_left_velocity_ = vel_rad_s;
        return vel_rad_s;
    }
    return last_left_velocity_;
}

double ZLAC8015DCanDriver::getRightVelocityRadianPerSec()
{
    uint8_t data[4];
    if (readSDO(ADDR_ACTUAL_VELOCITY, SUBINDEX_RIGHT_MOTOR, data, 4)) {
        int32_t raw_value = bytesToInt32(data);  // 单位：0.1 r/min
        double rpm = raw_value * 0.1;  // 转换为 r/min
        double vel_rad_s = rpm * RPM_TO_RAD_S;
        last_right_velocity_ = vel_rad_s;
        return -vel_rad_s;
    }
    return last_right_velocity_;
}

double ZLAC8015DCanDriver::getLeftPositionRadian()
{
    uint8_t data[4];
    if (readSDO(ADDR_ACTUAL_POSITION, SUBINDEX_LEFT_MOTOR, data, 4)) {
        int32_t counts = bytesToInt32(data);
        double radians = (counts / 17686.0) * 2.0 * M_PI;

        // 跳变检测
        if (left_position_initialized_) {
            double delta = std::fabs(radians - last_left_position_);
            if (delta > POSITION_JUMP_THRESHOLD) {
                // 异常跳变，返回缓存值
                std::cerr << "[ZLAC8015D] 左轮position跳变检测: delta=" << delta << " rad, 丢弃" << std::endl;
                return last_left_position_;
            }
        }

        // 更新缓存
        last_left_position_ = radians;
        left_position_initialized_ = true;
        return radians;
    }
    return last_left_position_;
}

double ZLAC8015DCanDriver::getRightPositionRadian()
{
    uint8_t data[4];
    if (readSDO(ADDR_ACTUAL_POSITION, SUBINDEX_RIGHT_MOTOR, data, 4)) {
        int32_t counts = bytesToInt32(data);
        double radians = -((counts / 17686.0) * 2.0 * M_PI);  // 右轮取反

        // 跳变检测
        if (right_position_initialized_) {
            double delta = std::fabs(radians - last_right_position_);
            if (delta > POSITION_JUMP_THRESHOLD) {
                // 异常跳变，返回缓存值
                std::cerr << "[ZLAC8015D] 右轮position跳变检测: delta=" << delta << " rad, 丢弃" << std::endl;
                return last_right_position_;
            }
        }

        // 更新缓存
        last_right_position_ = radians;
        right_position_initialized_ = true;
        return radians;
    }
    return last_right_position_;
}

bool ZLAC8015DCanDriver::sendSDO(uint16_t index, uint8_t subindex, const uint8_t* data, size_t len)
{
    if (can_socket_ < 0) return false;

    struct can_frame frame;
    frame.can_id = sdo_tx_;
    frame.can_dlc = 8;

    // 根据数据长度设置命令字
    if (len == 4) frame.data[0] = 0x23;      // 写4字节
    else if (len == 2) frame.data[0] = 0x2B;  // 写2字节
    else if (len == 1) frame.data[0] = 0x2F;  // 写1字节
    else return false;

    // 索引（小端）
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    // 子索引
    frame.data[3] = subindex;

    // 数据
    for (size_t i = 0; i < len && i < 4; i++) {
        frame.data[4 + i] = data[i];
    }
    for (size_t i = len; i < 4; i++) {
        frame.data[4 + i] = 0x00;
    }

    if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
        return false;
    }

    usleep(10000);
    return true;
}

bool ZLAC8015DCanDriver::readSDO(uint16_t index, uint8_t subindex, uint8_t* data, size_t len)
{
    if (can_socket_ < 0) return false;

    // 发送读请求
    struct can_frame frame_tx;
    frame_tx.can_id = sdo_tx_;
    frame_tx.can_dlc = 8;
    frame_tx.data[0] = 0x40;  // 读命令
    frame_tx.data[1] = index & 0xFF;
    frame_tx.data[2] = (index >> 8) & 0xFF;
    frame_tx.data[3] = subindex;
    frame_tx.data[4] = 0x00;
    frame_tx.data[5] = 0x00;
    frame_tx.data[6] = 0x00;
    frame_tx.data[7] = 0x00;

    if (write(can_socket_, &frame_tx, sizeof(frame_tx)) != sizeof(frame_tx)) {
        return false;
    }

    // *** 方案A关键修改：使用select()实现非阻塞超时读取 ***
    auto start_time = std::chrono::steady_clock::now();
    const int max_timeout_ms = 5;  // 降低超时到5ms，避免阻塞控制循环

    while (true) {
        // 检查总超时
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (elapsed >= max_timeout_ms) {
            return false;  // 超时，返回false，上层函数会使用缓存值
        }

        // 使用select检查是否有数据可读
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(can_socket_, &read_fds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 2000;  // 2ms select超时

        int select_ret = select(can_socket_ + 1, &read_fds, NULL, NULL, &timeout);

        if (select_ret < 0) {
            // select错误
            return false;
        } else if (select_ret == 0) {
            // select超时，继续循环直到总超时
            continue;
        }

        // 有数据可读，执行非阻塞读取
        struct can_frame frame_rx;
        ssize_t nbytes = read(can_socket_, &frame_rx, sizeof(frame_rx));

        if (nbytes != sizeof(frame_rx)) {
            // 读取失败（因为是非阻塞模式，EAGAIN/EWOULDBLOCK是正常的）
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;  // 没有数据，继续等待
            }
            return false;  // 真正的错误
        }

        // 验证CAN ID
        if (frame_rx.can_id != sdo_rx_) {
            continue;  // 不是SDO响应帧，丢弃
        }

        // 验证索引和子索引是否匹配
        bool index_match = (frame_rx.data[1] == (index & 0xFF)) &&
                          (frame_rx.data[2] == ((index >> 8) & 0xFF));
        bool subindex_match = (frame_rx.data[3] == subindex);

        if (!index_match || !subindex_match) {
            continue;  // 索引或子索引不匹配，丢弃继续读
        }

        // 找到匹配的响应帧，提取数据
        for (size_t i = 0; i < len && i < 4; i++) {
            data[i] = frame_rx.data[4 + i];
        }
        return true;
    }

    return false;
}

void ZLAC8015DCanDriver::int32ToBytes(int32_t value, uint8_t* bytes)
{
    bytes[0] = value & 0xFF;
    bytes[1] = (value >> 8) & 0xFF;
    bytes[2] = (value >> 16) & 0xFF;
    bytes[3] = (value >> 24) & 0xFF;
}

void ZLAC8015DCanDriver::uint32ToBytes(uint32_t value, uint8_t* bytes)
{
    bytes[0] = value & 0xFF;
    bytes[1] = (value >> 8) & 0xFF;
    bytes[2] = (value >> 16) & 0xFF;
    bytes[3] = (value >> 24) & 0xFF;
}

int32_t ZLAC8015DCanDriver::bytesToInt32(const uint8_t* bytes)
{
    int32_t value = bytes[0] |
                   (bytes[1] << 8) |
                   (bytes[2] << 16) |
                   (bytes[3] << 24);
    return value;
}
