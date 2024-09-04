
#pragma once
#include <smacc2/smacc.hpp>
#include <smacc2/client_bases/smacc_service_client.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cr_interfaces/srv/get_next_waypoint.hpp>

namespace sm_trial_planner
{
using namespace smacc2::client_bases;
class OrGetNextWaypoint : public smacc2::Orthogonal<OrGetNextWaypoint>
{
public:
    void onInitialize() override
    {
        auto service_client = this->createClient<SmaccServiceClient<cr_interfaces::srv::GetNextWaypoint>>("get_next_waypoint");
    }
};
} // namespace sm_trial_planner
