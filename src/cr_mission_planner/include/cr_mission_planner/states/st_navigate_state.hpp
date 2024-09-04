#pragma once

#include <smacc2/smacc.hpp>
#include <cr_mission_planner/sm_trial_planner.hpp>

namespace sm_trial_planner
{
struct NavigateState : smacc2::SmaccState<NavigateState, SmTrialPlanner>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  // typedef mpl::list<
  // use transition after reaching location to call the lowering mechanism down
  //     Transition<EvTimeElapsed<CbTimedSubscriber<std_msgs::msg::Float32>, OrSubscribeMeasurement>,
  //                GetNextWaypointState, SUCCESS>>
  //     reactions;

  static void staticConfigure()
  {
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    // GlobalBlackboard *bb;
    RCLCPP_INFO(getLogger(), "ListenAndService: Waiting for 100 seconds and message reception...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "ListenAndService: Waiting for 100 seconds and message reception...");
  }
};
}  // namespace sm_trial_planner
