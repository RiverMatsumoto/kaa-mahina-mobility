
#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <queue>
#include <utility>

struct GlobalBlackboard
{
  std::vector<std::pair<double, double>> gpsGrid; // .first is latitude, .second is longitude
  std::queue<geometry_msgs::msg::PoseStamped> gpsWaypointQueue;
  int gridLatitudeLength;
  int gridLongitudeLength;
  float averageMoisture;

  // Define the stream insertion operator for GlobalBlackboard
  friend std::ostream &operator<<(std::ostream &os, const GlobalBlackboard &blackboard)
  {
    os << "GlobalBlackboard { "
       << "gridLatitudeLength: " << blackboard.gridLatitudeLength
       << "gridLongitudeLength: " << blackboard.gridLongitudeLength
       << "averageMoisture: " << blackboard.averageMoisture;
    return os;
  }
};