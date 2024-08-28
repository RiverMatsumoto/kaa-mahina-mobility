
#include <cr_mission_planner/sm_trial_planner.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  smacc2::run<sm_trial_planner::SmTrialPlanner>();
}
