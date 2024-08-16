#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

class LoweringMechanism : public rclcpp::Node
{
public:
    PositionController()
    : Node("position_controller")
    {
        // Create a service that will trigger the movement
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "move_device", std::bind(&PositionController::move_device_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Position Controller Node has been started.");
    }

private:
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
        // Implement the logic to move the device up
        // Return true if the operation is successful, false otherwise
        try
        {
            RCLCPP_INFO(this->get_logger(), "Moving up...");
            // Simulate hardware interaction here
            return true;  // Assume success
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move up: %s", e.what());
            return false;
        }
    }

    bool move_down()
    {
        // Implement the logic to move the device down
        // Return true if the operation is successful, false otherwise
        try
        {
            RCLCPP_INFO(this->get_logger(), "Moving down...");
            // Simulate hardware interaction here
            return true;  // Assume success
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move down: %s", e.what());
            return false;
        }
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoweringMechanism>());
    rclcpp::shutdown();
    return 0;
}
