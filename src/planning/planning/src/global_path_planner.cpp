/*
 *  Global Path Planner Node
 *
 *  This node implements a global path planner for a car. It subscribes to
 *  the odometry and traffic light state topics, and uses a ROS service to set waypoints.
 *  These waypoints are organized in groups and are published sequentially based on the
 *  vehicle's current position and traffic light signals. The node ensures the waypoints
 *  are published only when the vehicle is in the correct state to proceed. The waypoints
 *  are used to generate a global path for the car to follow.
 *
 *  Author: Jincheng Pan
 *  Date: 2024-07-11
 */
#include <ros/ros.h>
#include "planning/Waypoint.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <vector>
#include <cmath>
#include "perception_msgs/TrafficState.h"

class GlobalPathPlanner
{
public:
  GlobalPathPlanner() : nh_(), car_x_(0.0), car_y_(0.0), trafficlight_signal_(false), waypoint_published_(false)
  {
    // Subscribe to the odometry topic to get current vehicle pose
    odom_sub_ = nh_.subscribe("/odom", 1, &GlobalPathPlanner::curPose, this);
    // Subscribe to the traffic light state topic
    trafficlight_state_sub_ =
        nh_.subscribe("/perception/traffic_state", 1, &GlobalPathPlanner::updateTrafficlightSignal, this);
    // Create a service client for setting waypoints
    client_ = nh_.serviceClient<planning::Waypoint>("/set_waypoint");
    // Wait for the waypoint service to be available
    ros::service::waitForService("/set_waypoint");

    // Initialize the waypoint groups
    initializeWaypoints();
  }

  void spin()
  {
    while (ros::ok())
    {
      // Check and publish waypoints if conditions are met
      checkAndPublishWaypoint();
      ros::spinOnce();
    }
  }

private:
  ros::NodeHandle nh_;                     // ROS node handle
  ros::Subscriber odom_sub_;               // Subscriber for odometry
  ros::Subscriber trafficlight_state_sub_; // Subscriber for traffic light state
  ros::ServiceClient client_;              // Service client for setting waypoints

  double car_x_;                                                  // Current X position of the car
  double car_y_;                                                  // Current Y position of the car
  double distance_to_part_goal_ = 0.0;                            // Distance to the current waypoint goal
  bool trafficlight_signal_;                                      // Current traffic light signal state
  bool waypoint_published_;                                       // Flag to check if waypoint is published
  size_t point_counter_ = 0;                                      // Counter for waypoints in the current group
  size_t group_counter_ = 0;                                      // Counter for waypoint groups
  std::vector<std::vector<double>> current_group_;                // Current group of waypoints
  std::vector<std::vector<std::vector<double>>> waypoint_groups_; // All waypoint groups

  // Callback to update the current position of the car
  void curPose(const nav_msgs::Odometry &cur_state)
  {
    car_x_ = cur_state.pose.pose.position.x;
    car_y_ = cur_state.pose.pose.position.y;
  }

  // Callback to update the traffic light signal state
  void updateTrafficlightSignal(const perception_msgs::TrafficState &trafficlight_state)
  {
    trafficlight_signal_ = trafficlight_state.state;
  }

  // Calculate the distance between two points
  double calculateDistance(double x1, double y1, double x2, double y2)
  {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  // Initialize the waypoint groups
  void initializeWaypoints()
  {
    waypoint_groups_ = {// Part 1
                        {
                            // 1
                            {-20.00, -60.40},
                            // 2
                            {-33.60, -57.70},
                            {-41.10, -52.40},
                            {-47.80, -41.96},
                            {-50.00, -33.600},
                            // 3
                            {-52.80, -14.32},
                            // 4
                            {-52.46, 39.64},
                            // 5 turn
                            {-52.50, 117.50},
                            {-48.30, 121.30},
                            // trafic light
                            {-10.26, 122.95},
                            {-10.00, 122.95},
                        },
                        // Part 2
                        {
                            // 6 turn
                            {-6.21, 123.00},
                            {-4.31, 127.00},
                            // trafic light
                            {-4.10, 222.60},
                            {-4.01, 223.10},
                        },
                        // Part 3
                        {
                            // 7 turn
                            {-5.10, 224.50},
                            {-7.73, 226.48},
                            // 8
                            {-29.54, 226.56},
                            // 9
                            {-37.97, 226.23},
                            {-44.62, 220.97},
                            {-48.98, 210.46},
                            {-50.98, 199.92},
                            // 5
                            {-52.20, 121.16},
                            // 10 turn
                            {-52.04, 50.51},
                            {-48.04, 46.51},
                            // traffic light
                            {-10.00, 45.32},
                            {-9.56, 45.32},
                        },
                        // Part 4
                        {
                            // 11 turn
                            {-8.01, 44.37},
                            {-4.41, 40.37},
                            // 12
                            {-3.84, 20.33},
                            // 13
                            {-3.00, -5.79},
                            // 14 traffic light
                            {-2.30, -59.00},
                            {-2.30, -59.50},
                        },
                        // Part 5
                        {
                            // 15
                            {-2.46, -63.00},
                            // 16
                            {4.74, -63.90},
                            {4.94, -63.90},
                        }};
  }

  // Check conditions and publish the next waypoint group if applicable
  void checkAndPublishWaypoint()
  {
    // If all waypoint groups have been published
    if (group_counter_ >= waypoint_groups_.size())
    {
      // ROS_INFO("All waypoint groups have been published.");
      return;
    }

    // Determine the current group of waypoints to process
    if (group_counter_ > 0)
    {
      current_group_ = waypoint_groups_[group_counter_ - 1];
      const auto &last_point_in_group = current_group_.back();
      distance_to_part_goal_ = calculateDistance(car_x_, car_y_, last_point_in_group[0], last_point_in_group[1]);
    }
    else
    {
      current_group_ = waypoint_groups_[0];
      const auto &last_point_in_group = current_group_.back();
      distance_to_part_goal_ = calculateDistance(car_x_, car_y_, last_point_in_group[0], last_point_in_group[1]);
    }

    // Check if we should publish the waypoint group
    if (group_counter_ == 0 && !trafficlight_signal_)
    {
      // Publish waypoints of the current group
      for (const auto &waypoint : current_group_)
      {
        planning::Waypoint goal;
        goal.request.posex_from_client = waypoint[0];
        goal.request.posey_from_client = waypoint[1];
        bool flag = client_.call(goal);
        // if (flag)
        // {
        //   ROS_INFO("Waypoint (%f, %f) successfully set!", waypoint[0], waypoint[1]);
        // }
        // else
        // {
        //   ROS_ERROR("Failed to set waypoint (%f, %f)!", waypoint[0], waypoint[1]);
        //   exit(1);
        // }
      }
      group_counter_++;
    }

    // Check if we should publish the next waypoint group
    if (group_counter_ > 0 && distance_to_part_goal_ <= 2.0 && !trafficlight_signal_)
    {
      current_group_ = waypoint_groups_[group_counter_];
      for (const auto &waypoint : current_group_)
      {
        planning::Waypoint goal;
        goal.request.posex_from_client = waypoint[0];
        goal.request.posey_from_client = waypoint[1];
        bool flag = client_.call(goal);
        // if (flag)
        // {
        //   ROS_INFO("Waypoint (%f, %f) successfully set!", waypoint[0], waypoint[1]);
        // }
        // else
        // {
        //   ROS_ERROR("Failed to set waypoint (%f, %f)!", waypoint[0], waypoint[1]);
        //   exit(1);
        // }
      }
      group_counter_++;
    }
  }
};

int main(int argc, char *argv[])
{
  // Initialize the ROS node
  ros::init(argc, argv, "global_path_planner");
  // Create an instance of GlobalPathPlanner
  GlobalPathPlanner planner;
  // Run the planner
  planner.spin();
  return 0;
}
