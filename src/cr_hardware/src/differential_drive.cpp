#include "cr_hardware/differential_drive.hpp"
#include "libroboclaw/roboclaw_driver.hpp"

#include <chrono>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
/*
ros2_control architecture notes:
- on_init: store the HardwareInfo for later use
*/

hardware_interface::CallbackReturn DifferentialDrive::on_init(
    const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    debug_ = info_.hardware_parameters["debug"].compare("True") == 0 ? true : false;
    RCLCPP_INFO(
        rclcpp::get_logger("DifferentialDrive"),
        "DEBUG MODE: %s", debug_ ? "True" : "False");

    // verify and error check joint types and interfaces
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("DiffBotSystemHardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("DiffBotSystemHardware"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("DiffBotSystemHardware"),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("DiffBotSystemHardware"),
                "Joint '%s' have %s state interfaces found. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("DiffBotSystemHardware"),
                "Joint '%s' have %s state interfaces found. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DifferentialDrive::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DifferentialDrive::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn DifferentialDrive::on_activate(
    const rclcpp_lifecycle::State &previous_state)
{
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        hw_positions_[i] = 0.0;
        hw_velocities_[i] = 0.0;
        hw_commands_[i] = 0.0;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("DifferentialDrive"),
        "Differential Drive has been activated.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DifferentialDrive::on_deactivate(
    const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(
        rclcpp::get_logger("DifferentialDrive"),
        "Differential Drive has been deactivated.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DifferentialDrive::read(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    // Read the state from the robot hardware
    // hw_positions_[0] = ...
    // hw_positions_[1] = ...
    // hw_velocities_[0] = ...
    // hw_velocities_[1] = ...

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DifferentialDrive::write(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    // Send the command to the robot hardware
    // hw_commands_[0] = ...
    // hw_commands_[1] = ...
    if (hw_commands_[0] != 0)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("DifferentialDrive"),
            "Sending velocity command to left wheel: %f", hw_commands_[0]);
    }
    if (hw_commands_[1] != 0)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("DifferentialDrive"),
            "Sending velocity command to right wheel: %f", hw_commands_[1]);
    }
    return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(DifferentialDrive, hardware_interface::SystemInterface)