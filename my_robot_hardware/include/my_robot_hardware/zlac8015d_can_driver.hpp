#ifndef ZLAC8015D_CAN_DRIVER_HPP
#define ZLAC8015D_CAN_DRIVER_HPP

#include <string>
#include <cstdint>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cmath>
#include <chrono>

// CANopen 对象字典地址
#define ADDR_HEARTBEAT          0x1017
#define ADDR_ASYNC_CONTROL      0x200F
#define ADDR_CONTROLWORD        0x6040
#define ADDR_OPERATION_MODE     0x6060
#define ADDR_TARGET_VELOCITY    0x60FF
#define ADDR_ACTUAL_VELOCITY    0x606C
#define ADDR_ACTUAL_POSITION    0x6064
#define ADDR_ACCEL_TIME         0x6083
#define ADDR_DECEL_TIME         0x6084
#define ADDR_QUICK_STOP_TIME    0x6085
#define ADDR_TORQUE_SLOPE       0x6087
// 厂家自定义参数地址
#define ADDR_MAX_CURRENT        0x2015
#define ADDR_OVERLOAD_RATIO     0x2012
#define ADDR_OVERLOAD_TIME      0x2016

// 电机子索引（ZLAC8015D双电机）
#define SUBINDEX_LEFT_MOTOR     0x01
#define SUBINDEX_RIGHT_MOTOR    0x02

// 转换常数
#define RPM_TO_RAD_S            (2.0 * M_PI / 60.0)  // RPM → rad/s
#define RAD_S_TO_RPM            (60.0 / (2.0 * M_PI))  // rad/s → RPM

// 跳变检测阈值（弧度）
#define POSITION_JUMP_THRESHOLD 2.0

/**
 * @brief ZLAC8015D双电机伺服控制器CAN驱动类
 *
 * 使用Linux SocketCAN接口与ZLAC8015D V4.0通讯
 * 支持CANopen DS301 & DS402协议
 */
class ZLAC8015DCanDriver {
public:
    /**
     * @brief 构造函数
     * @param can_interface CAN接口名称 (如 "can0")
     * @param node_id CANopen节点ID (默认1)
     */
    ZLAC8015DCanDriver(const std::string& can_interface, int node_id = 1);

    /**
     * @brief 析构函数
     */
    ~ZLAC8015DCanDriver();

    /**
     * @brief 初始化CAN接口
     * @return 0成功，-1失败
     */
    int init();

    /**
     * @brief 激活速度模式
     */
    void activateVelocityMode();

    /**
     * @brief 停用电机
     */
    void deactivate();

    /**
     * @brief 紧急停止
     */
    void emergencyStop();

    /**
     * @brief 设置加速时间
     * @param time_ms 加速时间（毫秒）
     */
    void setAccelerationTime(uint32_t time_ms);

    /**
     * @brief 设置减速时间
     * @param time_ms 减速时间（毫秒）
     */
    void setDecelerationTime(uint32_t time_ms);

    /**
     * @brief 设置扭矩斜率
     * @param slope_ma_s 扭矩斜率（mA/s）
     */
    void setTorqueSlope(uint32_t slope_ma_s);

    /**
     * @brief 设置目标速度（弧度/秒）
     * @param left_vel 左轮速度 (rad/s)
     * @param right_vel 右轮速度 (rad/s)
     */
    void setTargetVelocityRadianPerSec(double left_vel, double right_vel);

    /**
     * @brief 获取左轮速度（弧度/秒）
     * @return 左轮速度 (rad/s)
     */
    double getLeftVelocityRadianPerSec();

    /**
     * @brief 获取右轮速度（弧度/秒）
     * @return 右轮速度 (rad/s)
     */
    double getRightVelocityRadianPerSec();

    /**
     * @brief 获取左轮位置（弧度）
     * @return 左轮位置 (rad)
     */
    double getLeftPositionRadian();

    /**
     * @brief 获取右轮位置（弧度）
     * @return 右轮位置 (rad)
     */
    double getRightPositionRadian();

private:
    int can_socket_;              // SocketCAN套接字
    std::string can_interface_;   // CAN接口名称
    int node_id_;                 // CANopen节点ID
    uint32_t sdo_tx_;            // SDO发送COB-ID (0x600 + node_id)
    uint32_t sdo_rx_;            // SDO接收COB-ID (0x580 + node_id)

    // 位置累积（用于里程计）
    double left_position_;       // 左轮位置累积 (rad)
    double right_position_;      // 右轮位置累积 (rad)
    double last_left_velocity_;  // 上次左轮速度 (rad/s)
    double last_right_velocity_; // 上次右轮速度 (rad/s)

    // 位置缓存（用于跳变检测）
    double last_left_position_;  // 上次左轮位置 (rad)
    double last_right_position_; // 上次右轮位置 (rad)
    bool left_position_initialized_;   // 左轮位置是否已初始化
    bool right_position_initialized_;  // 右轮位置是否已初始化

    /**
     * @brief 发送SDO命令（写）
     * @param index 对象字典索引
     * @param subindex 子索引
     * @param data 数据指针
     * @param len 数据长度
     * @return true成功，false失败
     */
    bool sendSDO(uint16_t index, uint8_t subindex, const uint8_t* data, size_t len);

    /**
     * @brief 发送SDO命令（读）
     * @param index 对象字典索引
     * @param subindex 子索引
     * @param data 接收数据缓冲区
     * @param len 期望读取长度
     * @return true成功，false失败
     */
    bool readSDO(uint16_t index, uint8_t subindex, uint8_t* data, size_t len);

    /**
     * @brief 将int32转换为小端字节序
     * @param value 整数值
     * @param bytes 字节数组（至少4字节）
     */
    void int32ToBytes(int32_t value, uint8_t* bytes);

    /**
     * @brief 将uint32转换为小端字节序
     * @param value 无符号整数值
     * @param bytes 字节数组（至少4字节）
     */
    void uint32ToBytes(uint32_t value, uint8_t* bytes);

    /**
     * @brief 将小端字节序转换为int32
     * @param bytes 字节数组
     * @return 整数值
     */
    int32_t bytesToInt32(const uint8_t* bytes);
};

#endif // ZLAC8015D_CAN_DRIVER_HPP
