#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
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
        this->declare_parameter("rate_hz", 100);

        // store params
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
        this->get_parameter("rate_hz", rate_hz_);

        RCLCPP_INFO_STREAM(get_logger(), "Device: " << device_ << " Address: " << address_);

        RCLCPP_INFO_STREAM(get_logger(), "Using bno055 calibration offsets" << std::endl
                                                                            << "Accel offset x: " << accel_offset.x << std::endl
                                                                            << "Accel offset y: " << accel_offset.y << std::endl
                                                                            << "Accel offset z: " << accel_offset.z << std::endl
                                                                            << "Accel offset r: " << accel_offset.r << std::endl
                                                                            << "Gyro offset x: " << gyro_offset.x << std::endl
                                                                            << "Gyro offset y: " << gyro_offset.y << std::endl
                                                                            << "Gyro offset z: " << gyro_offset.z << std::endl
                                                                            << "Mag offset x: " << mag_offset.x << std::endl
                                                                            << "Mag offset y: " << mag_offset.y << std::endl
                                                                            << "Mag offset z: " << mag_offset.z << std::endl
                                                                            << "Mag offset r: " << mag_offset.r << std::endl
                                                                            );

        InitializeBNO055();

        bno055_write_accel_offset(&accel_offset);
        bno055_write_gyro_offset(&gyro_offset);
        bno055_write_mag_offset(&mag_offset);

        RCLCPP_INFO(get_logger(), "Finished initialization");
        RCLCPP_INFO_STREAM(get_logger(), "Started " << rate_hz_ << "hz imu readout");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&BNO055Node::ReadOrientation, this));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 20);
        abs_orientation_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("absolute_orientation", 20);
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
        bno055_linear_accel_float_t lin_accel = {0,0,0};
        bno055_gyro_float_t gyro = {0,0,0};
        bno055_quaternion_double_t quaternion = {0,0,0,0};
        // absolute orientation, heading pitch roll
        bno055_euler_float_t euler = {0,0,0};

        // +0 to +360 degrees for heading (0 should be north)
        // -180 to +180 degrees for pitch
        // -90 to +90 degrees for roll
        if (bno055_convert_float_linear_accel_xyz_msq(&lin_accel) != BNO055_SUCCESS)
            RCLCPP_ERROR(get_logger(), "Error reading linear acceleration data from imu");
        if (bno055_convert_float_gyro_xyz_rps(&gyro) != BNO055_SUCCESS)
            RCLCPP_ERROR(get_logger(), "Error reading gyro data from imu");
        bno055_convert_double_quaternion_wxyz(&quaternion); // no error code for some reason
        if (bno055_convert_float_euler_hpr_deg(&euler) != BNO055_SUCCESS)
            RCLCPP_ERROR(get_logger(), "Error reading euler absolute orientation from imu");
        
        // adjust for 0 = north
        int mag_offset_r;
        this->get_parameter("mag_offset.r", mag_offset_r);
        euler.h = euler.h - (mag_offset_r / 16.0f);

        // publish imu data
        sensor_msgs::msg::Imu imu_msg;
        geometry_msgs::msg::Vector3 abs_orientation_msg;
        imu_msg.header.frame_id = "bno055_imu_link";
        imu_msg.header.stamp = now();
        imu_msg.linear_acceleration.x = lin_accel.x;
        imu_msg.linear_acceleration.y = lin_accel.y;
        imu_msg.linear_acceleration.z = lin_accel.z;
        imu_msg.angular_velocity.x = gyro.x;
        imu_msg.angular_velocity.y = gyro.y;
        imu_msg.angular_velocity.z = gyro.z;
        imu_msg.orientation.x = quaternion.x;
        imu_msg.orientation.y = quaternion.y;
        imu_msg.orientation.z = quaternion.z;
        imu_msg.orientation.w = quaternion.w;
        abs_orientation_msg.x = euler.h;
        abs_orientation_msg.y = euler.p;
        abs_orientation_msg.z = euler.r;

        imu_pub_->publish(imu_msg);
        abs_orientation_pub_->publish(abs_orientation_msg);

        if (debug_)
        {
            RCLCPP_INFO(get_logger(), "Linear accel (m/s): X: %f, Y: %f, Z: %f", lin_accel.x, lin_accel.y, lin_accel.z);
            RCLCPP_INFO(get_logger(), "Gyro (rad/s): X: %f, Y: %f, Z: %f", gyro.x, gyro.y, gyro.z);
            RCLCPP_INFO(get_logger(), "Quaternion: X: %f, Y: %f, Z: %f", gyro.x, gyro.y, gyro.z);
            RCLCPP_INFO(get_logger(), "Euler orientation: Heading: %f, Pitch: %f, Roll: %f", euler.h, euler.p, euler.r);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::string device_;
    int address_;
    bno055_t bno055;
    bool debug_;
    int rate_hz_;

    // publishers
    // linear acceleration,
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr abs_orientation_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BNO055Node>());
    rclcpp::shutdown();
    return 0;
}
