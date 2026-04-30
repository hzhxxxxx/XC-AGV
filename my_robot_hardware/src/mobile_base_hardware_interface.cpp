#include "my_robot_hardware/mobile_base_hardware_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

namespace mobile_base_hardware{

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    // 读取CAN配置参数
    can_interface_ = info_.hardware_parameters["can_interface"];
    node_id_ = std::stoi(info_.hardware_parameters["node_id"]);

    // 创建CAN驱动
    driver_ = std::make_shared<ZLAC8015DCanDriver>(can_interface_, node_id_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    // 初始化CAN接口
    if (driver_->init() != 0) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    // 重置状态变量
    left_position_ = 0.0;
    left_velocity_ = 0.0;
    left_velocity_cmd_ = 0.0;
    right_position_ = 0.0;
    right_velocity_ = 0.0;
    right_velocity_cmd_ = 0.0;

    // 激活速度模式
    driver_->activateVelocityMode();

    // 等待电机状态稳定后再读取偏移值
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 记录初始编码器位置作为偏移量
    left_position_offset_ = driver_->getLeftPositionRadian();
    right_position_offset_ = driver_->getRightPositionRadian();
    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
        "Initial encoder offsets: left=%.2f rad, right=%.2f rad",
        left_position_offset_, right_position_offset_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    // 停用电机
    driver_->deactivate();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileBaseHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    // 从CAN驱动读取速度(原始值)
    double left_vel_raw = driver_->getLeftVelocityRadianPerSec();
    double right_vel_raw = driver_->getRightVelocityRadianPerSec();

    // 应用偏移量
    left_velocity_ = left_vel_raw;
    right_velocity_ = right_vel_raw;

    // 读取编码器绝对位置并减去初始偏移
    left_position_ = driver_->getLeftPositionRadian() - left_position_offset_;
    right_position_ = driver_->getRightPositionRadian() - right_position_offset_;

    return hardware_interface::return_type::OK;
}
hardware_interface::return_type MobileBaseHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    // 发送速度命令到CAN驱动
    driver_->setTargetVelocityRadianPerSec(left_velocity_cmd_, right_velocity_cmd_);

    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
  MobileBaseHardwareInterface::export_state_interfaces()
  {
      std::vector<hardware_interface::StateInterface> state_interfaces;

      // 驱动轮状态接口
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          "base_left_wheel_joint", "position", &left_position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          "base_left_wheel_joint", "velocity", &left_velocity_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          "base_right_wheel_joint", "position", &right_position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          "base_right_wheel_joint", "velocity", &right_velocity_));

      return state_interfaces;
  }
std::vector<hardware_interface::CommandInterface>
  MobileBaseHardwareInterface::export_command_interfaces()
  {
      std::vector<hardware_interface::CommandInterface> command_interfaces;

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          "base_left_wheel_joint", "velocity", &left_velocity_cmd_));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          "base_right_wheel_joint", "velocity", &right_velocity_cmd_));

      return command_interfaces;
  }

} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface,hardware_interface::SystemInterface)
