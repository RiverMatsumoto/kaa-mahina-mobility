#include <chrono>
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

        // offsets from bno055_driver.h
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
        this->declare_parameter<bool>("debug", false);
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
        this->get_parameter("debug", debug_);
        this->get_parameter("device", device_);
        this->get_parameter("address", address_);

        RCLCPP_INFO_STREAM(get_logger(), "Device: " << device_ << " Address: " << address_);

        RCLCPP_INFO_STREAM(get_logger(), "Using bno055 calibration offsets" << std::endl
                                                                            << "Accel offset x: " << accel_offset.x << std::endl
                                                                            << "Accel offset y: " << accel_offset.y << std::endl
                                                                            << "Accel offset z: " << accel_offset.z << std::endl
                                                                            << "Accel offset r: " << accel_offset.r);

        InitializeBNO055();

        bno055_write_accel_offset(&accel_offset);
        bno055_write_gyro_offset(&gyro_offset);
        bno055_write_mag_offset(&mag_offset);

        RCLCPP_INFO(get_logger(), "Started 1hz imu readout");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&BNO055Node::ReadOrientation, this));
    }

    ~BNO055Node()
    {
    }

private:
    void InitializeBNO055()
    {
        BNO055_I2C_init(device_.c_str(), address_);

        bno055.bus_read = BNO055_I2C_bus_read;
        bno055.bus_write = BNO055_I2C_bus_write;
        bno055.delay_msec = BNO055_delay_msec;
        bno055.dev_addr = address_;

        if (bno055_init(&bno055))
        {
            RCLCPP_INFO(get_logger(), "Error connecting. Will try connecting every second");
        }

        bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
        bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    }

    void ReadOrientation()
    {
        bno055_accel_t accel = {0,0,0};
        if (bno055_read_accel_xyz(&accel) != BNO055_SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Error reading from imu");
        }
        RCLCPP_INFO(get_logger(), "Accel (cm/s): X: %d, Y: %d, Z: %d", accel.x, accel.y, accel.z);

        bno055_euler_t euler = {0,0,0};
        if (bno055_read_euler_hrp(&euler) != BNO055_SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Error reading from imu");
        }
        RCLCPP_INFO(get_logger(), "Euler (degrees): Heading: %d, Roll: %d, Pitch: %d", euler.h, euler.r, euler.p);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::string device_;
    int address_;
    bno055_t bno055;
    bool debug_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BNO055Node>());
    rclcpp::shutdown();
    return 0;
}
