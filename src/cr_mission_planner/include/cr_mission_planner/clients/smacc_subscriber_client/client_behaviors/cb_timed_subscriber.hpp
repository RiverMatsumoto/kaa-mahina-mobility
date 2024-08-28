
#pragma once

#include <smacc2/smacc.hpp>
#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <cr_mission_planner/global_blackboard.hpp>
#include <cr_interfaces/srv/get_next_waypoint.hpp>
#include <std_msgs/msg/float32.hpp>

namespace sm_trial_planner
{
// EVENTS
template <typename TSource, typename TOrthogonal>
struct EvTimeElapsed : sc::event<EvTimeElapsed<TSource, TOrthogonal>> {};

// template <typename TSource, typename TOrthogonal>
// struct EvTimer : sc::event<EvTimer<TSource, TOrthogonal>>

template <typename MessageType>
class CbTimedSubscriber : public smacc2::SmaccClientBehavior
{

public:
  explicit CbTimedSubscriber(rclcpp::Duration timeout) : timeoutDuration_(timeout) {}

  void onEntry() override
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "CbTimedSubscribe: onEntry");

    // Get the subscriber client
    this->requiresClient(subscriberClient_);
    subscriberClient_->onMessageReceived(&CbTimedSubscriber<MessageType>::onMessageReceived, this);
    subscriberClient_->onFirstMessageReceived(&CbTimedSubscriber<MessageType>::onFirstMessageReceived, this);

    startTime_ = this->getNode()->get_clock()->now();
  }

  void onExit() override
  {
    RCLCPP_INFO(this->getNode()->get_logger(), "CbTimedSubscribe: onExit");
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->postTimeElapsedEvent_ = [=]()
    {
      this->template postEvent<EvTimeElapsed<TSourceObject, TOrthogonal>>();
    };
  }

protected:
  virtual void onMessageReceived(const MessageType &msg)
  {
    // RCLCPP_INFO_STREAM(getNode()->get_logger(), "CbTimedSubscriber Duration: " << (startTime_ - this->getNode()->get_clock()->now()));

    auto timeElapsed = this->getNode()->get_clock()->now() - startTime_;
    RCLCPP_INFO_STREAM(getNode()->get_logger(), "Duration in seconds: " << timeElapsed.seconds());

    if (timeElapsed < timeoutDuration_)
    {
      RCLCPP_INFO(getNode()->get_logger(), "CbTimedSubscribe: Received message");
    }
    else
    {
      // Wait for the service to be available
      // while (!client_->wait_for_service(std::chrono::seconds(1))) {
      //   RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
      // }

      // auto request = std::make_shared<cr_mission_planner::srv::GetNextWaypoint::Request>();
      // request->threshold = 1.5f;  // Example threshold value

      // auto result_future = client_->async_send_request(request);
      
      // // Wait for the result
      // if (rclcpp::spin_until_future_complete(getNode()->get_node_base_interface(), result_future) ==
      //     rclcpp::FutureReturnCode::SUCCESS)
      // {
      //   auto response = result_future.get();
      //   RCLCPP_INFO(this->get_logger(), "Received response: column = %d, row = %d", response->column, response->row);
      // } 
      // else
      // {
      //   RCLCPP_ERROR(this->get_logger(), "Failed to call service");
      // }
      postTimeElapsedEvent_();
    }
  }

  virtual void onFirstMessageReceived(const MessageType &msg)
  {
    RCLCPP_INFO(getNode()->get_logger(), "CbTimedSubscribe: First Message Received!");
  }

  void requestNextWaypoint()
  {
    RCLCPP_INFO(getNode()->get_logger(), "CbTimedSubscribe: Requesting next waypoint...");
  }

private:
  smacc2::client_bases::SmaccSubscriberClient<MessageType> *subscriberClient_;
  rclcpp::Duration timeoutDuration_;
  rclcpp::Time startTime_;
  std::function<void()> postTimeElapsedEvent_;
};

} // namespace sm_trial_planner
