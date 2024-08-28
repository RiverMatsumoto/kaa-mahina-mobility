
#include <smacc2/smacc.hpp>
#include <cr_mission_planner/global_blackboard.hpp>
#include <std_msgs/msg/float32.hpp>

//
// #include <smacc2/client_bases/smacc_subscriber_client.hpp>
// #include <std_msgs/msg/float32.hpp>

namespace sm_trial_planner
{
using namespace smacc2::default_transition_tags;

struct MeasureState : smacc2::SmaccState<MeasureState, SmTrialPlanner>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvTimeElapsed<CbTimedSubscriber<std_msgs::msg::Float32>, OrSubscribeMeasurement>, NavigateState, SUCCESS>
    >reactions;

    static void staticConfigure()
    {
        configure_orthogonal<OrSubscribeMeasurement, CbTimedSubscriber<std_msgs::msg::Float32>>(rclcpp::Duration::from_seconds(5));
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
} // namespace sm_trial_planner