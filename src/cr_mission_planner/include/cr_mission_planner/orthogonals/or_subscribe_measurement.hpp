
#include <smacc2/smacc.hpp>
#include <smacc2/client_bases/smacc_service_client.hpp>
#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cr_interfaces/srv/get_next_waypoint.hpp>

namespace sm_trial_planner
{
using namespace smacc2::client_bases;
class OrSubscribeMeasurement : public smacc2::Orthogonal<OrSubscribeMeasurement>
{
public:
    void onInitialize() override
    {
        auto client = this->createClient<SmaccSubscriberClient<std_msgs::msg::Float32>>("moisture_percent");
        auto service_client = this->createClient<SmaccServiceClient<cr_interfaces::srv::GetNextWaypoint>>("get_next_waypoint");
        // client->initialize();
    }
};
} // namespace sm_trial_planner