
#pragma once

// for gps to map frame conversion
#include <geodesy/utm.h>
#include <geographic_msgs/msg/geo_point.hpp>

#include <geographic_msgs/msg/geo_pose_stamped.hpp>

#include <vector>
#include <cmath>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include "global_blackboard.hpp"

const double R = 6378137.0; // radius of earth

double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

double toDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

// calculate poisition by aiming towards the bearing and moving forward in that distance
std::pair<double, double> move_point(double lat, double lon, double distance, double bearing) {
    double lat_rad = toRadians(lat);
    double lon_rad = toRadians(lon);
    double bearing_rad = toRadians(bearing);

    // New latitude in radians
    double new_lat_rad = std::asin(std::sin(lat_rad) * std::cos(distance / R) +
                                   std::cos(lat_rad) * std::sin(distance / R) * std::cos(bearing_rad));

    // New longitude in radians
    double new_lon_rad = lon_rad + std::atan2(std::sin(bearing_rad) * std::sin(distance / R) * std::cos(lat_rad),
                                              std::cos(distance / R) - std::sin(lat_rad) * std::sin(new_lat_rad));

    // Convert back to degrees
    double new_lat = toDegrees(new_lat_rad);
    double new_lon = toDegrees(new_lon_rad);

    return std::make_pair(new_lat, new_lon);
}

// generate grid of gps coordinates, each point is calculated by moving forward, then rotating and then moving forward again (along horizontal axis)
std::vector<std::pair<double, double>> generate_gps_grid(double initial_lat, double initial_lon, double initial_bearing,
                                                         int num_points_forward, int num_points_right, double spacing_meters = 2.0) {
    std::vector<std::pair<double, double>> grid;

    for (int i = 0; i < num_points_forward; ++i) {
        for (int j = 0; j < num_points_right; ++j) {
            // Calculate the forward movement
            auto [forward_lat, forward_lon] = move_point(initial_lat, initial_lon, i * spacing_meters, initial_bearing);
            // Calculate the rightward movement
            auto [right_lat, right_lon] = move_point(forward_lat, forward_lon, j * spacing_meters, fmod(initial_bearing + 90.0, 360.0));
            grid.emplace_back(right_lat, right_lon);
        }
    }

    return grid;
}

// Assuming you have a function that converts GPS (latitude, longitude) to map frame coordinates
void gpsToMapFrame(double latitude, double longitude, double &x, double &y);

void addWaypoint(double latitude, double longitude, double altitude, GlobalBlackboard &bb, std::string frame_id = "map")
{
    geometry_msgs::msg::PoseStamped waypoint;

    // Convert GPS coordinates to map frame coordinates
    double x, y;
    gpsToMapFrame(latitude, longitude, x, y);

    // Set the pose position
    waypoint.pose.position.x = x;
    waypoint.pose.position.y = y;
    waypoint.pose.position.z = altitude; // Typically use altitude from GPS, or set to 0 if irrelevant

    // Set the orientation (facing forward, typically no rotation)
    waypoint.pose.orientation.w = 1.0; // Identity quaternion (no rotation)
    waypoint.pose.orientation.x = 0.0;
    waypoint.pose.orientation.y = 0.0;
    waypoint.pose.orientation.z = 0.0;

    // Set the frame ID and timestamp
    waypoint.header.frame_id = frame_id;
    waypoint.header.stamp = rclcpp::Clock().now();

    // Add to the waypoint queue
    // waypointQueue.push(waypoint);
}

void gpsToMapFrame(double latitude, double longitude, double &x, double &y)
{
    geographic_msgs::msg::GeoPoint geo_point;
    geo_point.latitude = latitude;
    geo_point.longitude = longitude;

    geodesy::UTMPoint utm_point(geo_point);
    x = utm_point.easting;
    y = utm_point.northing;
}
