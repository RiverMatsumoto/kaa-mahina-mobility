#ifndef DIFFERENTIAL_DRIVE_HARDWARE_HPP
#define DIFFERENTIAL_DRIVE_HARDWARE_HPP

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

#include "cr_hardware/roboclaw.h"

class DifferentialDrive : public hardware_interface::SystemInterface
                                    // public std::enable_shared_from_this<DifferentialDrive>
{
public:
    // populate ros2_control interface info. initialize ros2_control hardware state and command interfaces
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;
    
    // allocate and deallocate memory for hardware controls
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    roboclaw *rc_;
    double hw_start_sec_;
    double hw_stop_sec_;
    std::string serial_port_;
    int baud_rate_;

    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    bool debug_;
    double wheel_radius_;
    double circumference_;
    int last_speed_cmd_left_;
    int last_speed_cmd_right_;
    const double enc_ticks_per_revolution_ = 610;
    const double revolutions_per_enc_tick_ = 1 / 610;
};

#endif
