
#pragma once
#include <nav2z_client/nav2z_client.hpp>
#include <smacc2/smacc.hpp>

namespace sm_trial_planner
{
class OrNavigate : public smacc2::Orthogonal<OrNavigate>
{
public:
    void onInitialize() override
    {
        auto client = this->createClient<cl_nav2z::ClNav2Z>();
    }
};
} // namespace sm_trial_planner
