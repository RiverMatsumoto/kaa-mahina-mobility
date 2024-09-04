
#pragma once

#include <smacc2/smacc.hpp>
#include <smacc2/client_bases/smacc_service_client.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cr_interfaces/srv/get_next_waypoint.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace sm_trial_planner
{
using namespace smacc2::client_bases;
class OrLoweringMechanism : public smacc2::Orthogonal<OrLoweringMechanism>
{
public:
    void onInitialize() override
    {
        auto service_client = this->createClient<SmaccServiceClient<std_srvs::srv::SetBool>>("lower_probe");
    }
};
} // namespace sm_trial_planner
