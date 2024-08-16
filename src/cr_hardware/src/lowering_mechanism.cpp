#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

// roboclaw hardware control
#include "cr_hardware/roboclaw.h"

class LoweringMechanism : public rclcpp::Node
{
public:
    LoweringMechanism()
    : Node("lowering_mechanism")
    {
        // Create a service that will trigger the movement
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "move_device", std::bind(&LoweringMechanism::move_device_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Position Controller Node has been started.");
    }

private:
    // need init function
    void init_roboclaw()
    {
        rc_ = roboclaw_init(tty_device_, baud_rate_);
        if (rc_ == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Roboclaw. Check for power and communication. Shutting down");
            rclcpp::shutdown();
        }
    }

    void move_device_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        bool success = false;

        if (request->data)  // True means "up"
        {
            success = move_up();
            response->message = success ? "Moved Up" : "Failed to Move Up";
        }
        else  // False means "down"
        {
            success = move_down();
            response->message = success ? "Moved Down" : "Failed to Move Down";
        }

        response->success = success;
        RCLCPP_INFO(this->get_logger(), "Action: %s | Success: %d", request->data ? "Up" : "Down", success);
    }

    bool move_up()
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "Moving up...");
            if (roboclaw_move_to_position_m1(rc_, address_, 0) != ROBOCLAW_OK)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to move up");
                return false;
            }
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move up, likely lost connection with roboclaw: %s", e.what());
            return false;
        }
    }

    bool move_down()
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "Moving down...");
            if (roboclaw_move_to_position_m1(rc_, address_, 1000) != ROBOCLAW_OK)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to move up");
                return false;
            }
            return true;  // Assume success
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move down, likely lost connection with roboclaw: %s", e.what());
            return false;
        }
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    const char *tty_device_ = "/dev/ttyUSB0";
    const int address_ = 130;
    const int baud_rate_ = 115200;
    struct roboclaw *rc_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoweringMechanism>());
    rclcpp::shutdown();
    return 0;
}
