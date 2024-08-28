
#include <smacc2/smacc.hpp>

// CLIENTS
#include <nav2z_client/nav2z_client.hpp> 
#include <smacc2/client_bases/smacc_subscriber_client.hpp> // used to subscribe to probe measurements

// CLIENT BEHAVIORS
#include <nav2z_client/client_behaviors/cb_navigate_global_position.hpp>
#include <cr_mission_planner/clients/smacc_subscriber_client/client_behaviors/cb_timed_subscriber.hpp>

// ORTHOGONALS
#include <cr_mission_planner/orthogonals/or_subscribe_measurement.hpp>

// BLACKBOARD
#include <cr_mission_planner/global_blackboard.hpp>

// MISSION PLANNER UTILS
#include <cr_mission_planner/generate_gps_grid.hpp>

using namespace boost;
using namespace smacc2;

namespace sm_trial_planner
{
// STATES
class NavigateState;
class MeasureState;

// STATE MACHINE
struct SmTrialPlanner : public smacc2::SmaccStateMachineBase<SmTrialPlanner, MeasureState>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        GlobalBlackboard blackboard;
        // Initialize gps grid and global blackboard
        this->getNode()->declare_parameter<double>("initial_latitude", 0.0);
        this->getNode()->declare_parameter<double>("initial_longitude", 0.0);
        this->getNode()->declare_parameter<double>("initial_bearing", 0.0); // 0 is north, 45 degrees is northeast
        this->getNode()->declare_parameter<int>("grid_latitude_length", 5.0);
        this->getNode()->declare_parameter<int>("grid_longitude_length", 5.0);

        double initialLat, initialLon, initialBearing;
        int gridLatLength, gridLonLength;

        this->getNode()->get_parameter("initialLatitude", initialLat);
        this->getNode()->get_parameter("initialLongitude", initialLon);
        this->getNode()->get_parameter("initialBearing", initialBearing);
        this->getNode()->get_parameter("gridLatitudeLength", gridLatLength);
        this->getNode()->get_parameter("gridLongitudeLength", gridLonLength);

        blackboard.gpsGrid = generate_gps_grid(initialLat, initialLon, initialBearing, gridLatLength, gridLonLength);
        this->setGlobalSMData("blackboard", blackboard);
        this->createOrthogonal<OrSubscribeMeasurement>();
    }
};

} // namespace sm_trial_planner


#include <cr_mission_planner/states/st_measure_state.hpp>
#include <cr_mission_planner/states/st_navigate_state.hpp>