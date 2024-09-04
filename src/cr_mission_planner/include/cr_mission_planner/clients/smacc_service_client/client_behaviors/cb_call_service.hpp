
#pragma once

#include "smacc2/client_bases/smacc_service_client.hpp"
#include <rclcpp/logging.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/impl/smacc_asynchronous_client_behavior_impl.hpp>
#include <smacc2/smacc_client_behavior.hpp>
#include <std_msgs/msg/float32.hpp>

namespace sm_trial_planner
{
using namespace std::chrono_literals;
using namespace std_msgs::msg;
using namespace smacc2::client_bases;

struct EvServiceCallSuccess : sc::event<EvServiceCallSuccess> {};
struct EvServiceCallFailure : sc::event<EvServiceCallFailure> {};

template <typename ServiceType> class CbServiceCall : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbServiceCall(const char *serviceName) : serviceName_(serviceName)
  {
    request_ = std::make_shared<typename ServiceType::Request>();
    pollRate_ = 100ms;
  }

  CbServiceCall(const char *serviceName, std::shared_ptr<typename ServiceType::Request> request,
                std::chrono::milliseconds pollRate = 100ms)
      : serviceName_(serviceName), request_(request), result_(nullptr), pollRate_(pollRate)
  {
  }


  void setRequest(const std::shared_ptr<typename ServiceType::Request> &request)
  {
    request_ = request;
  }

  void onEntry() override
  {
  }

  void call()
  {
    RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "]" <<" Making service call");
    this->requiresClient(serviceClient_);
    std::shared_ptr<typename ServiceType::Response> response = serviceClient_->call(request_);
    if (!response)
    {
      RCLCPP_ERROR(getLogger(), "Service call failed, received null response!");
      this->postEvent<EvServiceCallFailure>();
      return;
    }
    onServiceResponse(result_);
    evServiceCallSuccessSignal_();
    this->postEvent<EvServiceCallSuccess>();
  }

  std::shared_future<std::shared_ptr<typename ServiceType::Response>> resultFuture_;
  typename std::shared_ptr<typename ServiceType::Response> result_;
  std::chrono::milliseconds pollRate_;
  boost::signals2::signal<void()> evServiceCallSuccessSignal_;

protected:
  SmaccServiceClient<ServiceType> *serviceClient_;
  std::string serviceName_;
  std::shared_ptr<typename ServiceType::Request> request_;

  virtual void onServiceResponse(std::shared_ptr<typename ServiceType::Response> result)
  {
    RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] " << result << " response received ");
  }
};

} // namespace sm_trial_planner
