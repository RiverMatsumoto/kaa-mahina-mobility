
#pragma once

#include <chrono>
#include <cr_interfaces/srv/detail/get_next_waypoint__struct.hpp>
#include <memory>
#include <thread>

// CLIENT BEHAVIORS
#include <cr_mission_planner/clients/smacc_subscriber_client/client_behaviors/cb_timed_subscriber.hpp>
#include <cr_mission_planner/clients/smacc_service_client/client_behaviors/cb_call_service.hpp>

// ORTHOGONALS
#include <cr_mission_planner/orthogonals/or_lowering_mechanism.hpp>

// SERVICES
#include <cr_interfaces/srv/get_next_waypoint.hpp>
#include <std_srvs/srv/set_bool.hpp>

// MESSAGES
#include <std_msgs/msg/float32.hpp>

#include <cr_mission_planner/global_blackboard.hpp>
#include <cr_mission_planner/sm_trial_planner.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_transition.hpp>

namespace sm_trial_planner
{
using namespace smacc2::default_transition_tags;
using namespace std_msgs::msg;
using namespace std_srvs::srv;
using namespace cr_interfaces::srv;

// lower probe -> take measurements for 100 seconds
struct MeasureState : smacc2::SmaccState<MeasureState, SmTrialPlanner>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
      Transition<EvTimeElapsed<
        CbTimedSubscriber<std_msgs::msg::Float32>, OrSubscribeMeasurement>,
        GetNextWaypointState,
        SUCCESS>
      >reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrLoweringMechanism, CbServiceCall<SetBool>>("lower_probe");
    configure_orthogonal<OrSubscribeMeasurement, CbTimedSubscriber<std_msgs::msg::Float32>>(
        rclcpp::Duration::from_seconds(5));
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    // GlobalBlackboard *bb;

    // this->getStateMachine().createSignalConnection();
    auto lowerProbeServiceCb = this->getClientBehavior<OrLoweringMechanism, CbServiceCall<SetBool>>();
    auto request = std::make_shared<SetBool::Request>();
    lowerProbeServiceCb->setRequest(request);
    this->getStateMachine().createSignalConnection(
        lowerProbeServiceCb->evServiceCallSuccessSignal_,
        &MeasureState::onServiceCallCompleted,
        this);
    lowerProbeServiceCb->call();

    RCLCPP_INFO(getLogger(), "Lowering Probe");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "ListenAndService: Waiting for 100 seconds and message reception...");
  }

  void onServiceCallCompleted()
  {
    RCLCPP_INFO(getLogger(), "Waiting for 2 seconds to start listening to measurements");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(getLogger(), "Listening to probe measurements");
    auto cbSubscribeMeasurement = this->getClientBehavior<OrSubscribeMeasurement, CbTimedSubscriber<Float32>>();
    cbSubscribeMeasurement->startTimerAndSubscribing();
  }

  boost::signals2::signal<void()> evServiceCallSuccessSignal;
};
} // namespace sm_trial_planner
