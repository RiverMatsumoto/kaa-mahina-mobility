
#include <smacc2/smacc.hpp>

namespace sm_trial_planner
{
struct NavigateState : smacc2::SmaccState<NavigateState, SmTrialPlanner>
{
  using SmaccState::SmaccState;

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "DEBUG NavigateState: State Machine will exit now.");
  }
};
}  // namespace sm_trial_planner
