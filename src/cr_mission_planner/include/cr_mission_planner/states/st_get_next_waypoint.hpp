
#pragma once

#include <cr_mission_planner/orthogonals/or_get_next_waypoint.hpp>
#include <cr_interfaces/srv/detail/get_next_waypoint__struct.hpp>
#include <cr_interfaces/srv/get_next_waypoint.hpp>
#include <cr_mission_planner/clients/smacc_service_client/client_behaviors/cb_call_service.hpp>
#include <cr_mission_planner/global_blackboard.hpp>
#include <cr_mission_planner/sm_trial_planner.hpp>
#include <memory>
#include <smacc2/smacc.hpp>
#include <std_msgs/msg/float32.hpp>

namespace sm_trial_planner
{
using namespace smacc2::default_transition_tags;
using namespace std_msgs::msg;

struct GetNextWaypointState : smacc2::SmaccState<GetNextWaypointState, SmTrialPlanner>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbServiceCall<cr_interfaces::srv::GetNextWaypoint>, OrGetNextWaypoint>, NavigateState, SUCCESS>>
  reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrGetNextWaypoint,
                         CbServiceCall<cr_interfaces::srv::GetNextWaypoint>>("get_next_waypoint");
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    // GlobalBlackboard *bb;
    RCLCPP_INFO(getLogger(), "Getting next waypoint...");
    auto cbService = this->getClientBehavior<OrGetNextWaypoint, CbServiceCall<cr_interfaces::srv::GetNextWaypoint>>();
    auto request = std::make_shared<cr_interfaces::srv::GetNextWaypoint::Request>();

    cbService->setRequest(request);
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "Exiting GetNextWaypointState.");
  }
};
} // namespace sm_trial_planner
