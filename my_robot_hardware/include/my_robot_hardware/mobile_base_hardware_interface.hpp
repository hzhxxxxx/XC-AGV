#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "my_robot_hardware/zlac8015d_can_driver.hpp"

namespace mobile_base_hardware{

class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // Lifecycle node override
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // SystemInterface override
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    std::shared_ptr<ZLAC8015DCanDriver> driver_;
    std::string can_interface_;
    int node_id_;

    // 初始编码器位置（用于计算相对位移）
    double left_position_offset_ = 0.0;
    double right_position_offset_ = 0.0;
    double left_velocity_offset_ = 0.0;
    double right_velocity_offset_ = 0.0;
    // 状态变量
    double left_position_ = 0.0;
    double left_velocity_ = 0.0;
    double left_velocity_cmd_ = 0.0;
    double right_position_ = 0.0;
    double right_velocity_ = 0.0;
    double right_velocity_cmd_ = 0.0;

    // 读取计数器（用于限制日志输出）
    int read_count_ = 0;

};

} // namespace mobile_base_hardware

#endif
