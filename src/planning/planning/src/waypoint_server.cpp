/*
 *  Waypoint Server Node
 *
 *  This node provides a ROS service to set waypoints and publishes the received
 *  waypoints to the "waypoints_list" topic. It uses a service callback to handle incoming requests
 *  and publishes the waypoints as PointStamped messages.
 *
 *  Author: 
 *  Date: 2024-07-01
 */
#include <ros/ros.h>
#include "planning/Waypoint.h"
#include "geometry_msgs/PointStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

class WaypointServer
{
public:
  // Constructor
  WaypointServer()
    : nh_()  
    , serv_(nh_.advertiseService("/set_waypoint", &WaypointServer::doRequ, this))  
    , pub_(nh_.advertise<geometry_msgs::PointStamped>("/waypoints_list", 1))  
  {
  }

  // Service callback function to handle incoming requests
  bool doRequ(planning::Waypoint::Request& req, planning::Waypoint::Response& resp)
  {
    try
    {
      // Get the pose from the request and set it in the response
      resp.posex = req.posex_from_client;
      resp.posey = req.posey_from_client;
      resp.quaternionz = req.quaternionz_from_client;
      resp.quaternionw = req.quaternionw_from_client;

      // Set the waypoint data
      waypoint_.point.x = resp.posex;
      waypoint_.point.y = resp.posey;
      waypoint_.point.z = 0.0;  
      waypoint_.header.frame_id = "world";  // Set the frame of reference

      // Publish the waypoint
      pub_.publish(waypoint_);
    }
    catch (const std::exception& e)
    {
      // Log an error message if an exception occurs
      ROS_ERROR("Server encountered an error: %s", e.what());
      return false;
    }
    // Indicate successful handling of the request
    return true;
  }

private:
  geometry_msgs::PointStamped waypoint_;  // Message to store waypoint data
  ros::NodeHandle nh_;  // ROS node handle
  ros::ServiceServer serv_;  // ROS service server
  ros::Publisher pub_;  // ROS publisher
};

int main(int argc, char* argv[])
{
  // Initialize the ROS node
  ros::init(argc, argv, "waypoint_server");
  // Create an instance of the WaypointServer
  WaypointServer server;
  ROS_INFO("Waypoint server started.");
  // Spin to keep the node running and processing callbacks
  ros::spin();
}
