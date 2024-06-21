#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "bno055_driver.h"

class BNO055Node : public rclcpp::Node
{
public:
    BNO055Node() : Node("bno055_node")
    {
        RCLCPP_INFO(get_logger(), "Initializing BNO055 9DOF Sensor");
        bno055_accel_offset_t accel_offset;
        bno055_gyro_offset_t gyro_offset;
        bno055_mag_offset_t mag_offset;

        // using types from bno055_driver.h
        this->declare_parameter<s16>("accel_offset.x", 0);
        this->declare_parameter<s16>("accel_offset.y", 0);
        this->declare_parameter<s16>("accel_offset.z", 0);
        this->declare_parameter<s16>("accel_offset.r", 0);
        this->declare_parameter<s16>("gyro_offset.x", 0);
        this->declare_parameter<s16>("gyro_offset.y", 0);
        this->declare_parameter<s16>("gyro_offset.z", 0);
        this->declare_parameter<s16>("mag_offset.x", 0);
        this->declare_parameter<s16>("mag_offset.y", 0);
        this->declare_parameter<s16>("mag_offset.z", 0);
        this->declare_parameter<s16>("mag_offset.r", 0);
        // device config, jetson orin nano uses bus 7, bno055 default address is 0x28
        this->declare_parameter("device", "/dev/i2c-7");
        this->declare_parameter("address", 0x28);

        this->get_parameter("accel_offset.x", accel_offset.x);
        this->get_parameter("accel_offset.y", accel_offset.y);
        this->get_parameter("accel_offset.z", accel_offset.z);
        this->get_parameter("accel_offset.r", accel_offset.r);
        this->get_parameter("gyro_offset.x", gyro_offset.x);
        this->get_parameter("gyro_offset.y", gyro_offset.y);
        this->get_parameter("gyro_offset.z", gyro_offset.z);
        this->get_parameter("mag_offset.x", mag_offset.x);
        this->get_parameter("mag_offset.y", mag_offset.y);
        this->get_parameter("mag_offset.z", mag_offset.z);
        this->get_parameter("mag_offset.r", mag_offset.r);
        this->get_parameter("device", device_);
        this->get_parameter("address", address_);

        RCLCPP_INFO_STREAM(get_logger(), "Using bno055 calibration offsets" << 
            std::endl << "Accel offset x: " << accel_offset.x <<
            std::endl << "Accel offset y: " << accel_offset.y <<
            std::endl << "Accel offset z: " << accel_offset.z <<
            std::endl << "Accel offset r: " << accel_offset.r);

        InitializeBNO055();

        this->create_wall_timer(std::chrono::seconds(1), std::bind(&BNO055Node::ReadOrientation, this));
    }

private:
    void InitializeBNO055()
    {
        BNO055_I2C_init(device_.c_str(), address_);
    }

    void ReadOrientation()
    {
        struct bno055_euler_float_t euler;
        bno055_accel_t accel;
        bno055_read_accel_xyz(&accel);
        RCLCPP_INFO(get_logger(), "Euler: Heading: %f, Roll: %f, Pitch: %f", euler.h, euler.r, euler.p);
    }

    std::string device_;
    int address_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BNO055Node>());

    return 0;
}
