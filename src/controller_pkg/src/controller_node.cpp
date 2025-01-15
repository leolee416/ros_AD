#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>

#define PI M_PI

class controllerNode {
  ros::NodeHandle nh;

  ros::Subscriber current_state_sub;
  ros::Subscriber ackermann_cmd_sub;
  ros::Publisher car_commands_pub;
  ros::Timer timer;

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  // Ackermann command
  double ackermann_speed;
  double ackermann_steering_angle;

public:
  controllerNode() : hz(50.0), ackermann_speed(0.0), ackermann_steering_angle(0.0) {
    // current_state_sub = nh.subscribe("odom", 1, &controllerNode::onCurrentState, this);
    ackermann_cmd_sub = nh.subscribe("/pid_cmd", 1, &controllerNode::onPidCmd, this);
    car_commands_pub = nh.advertise<mav_msgs::Actuators>("/car_commands", 1);
    timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
  }

  // void onCurrentState(const nav_msgs::Odometry& cur_state) {
  //   x << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
  //   v << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y, cur_state.twist.twist.linear.z;
  //   omega << cur_state.twist.twist.angular.x, cur_state.twist.twist.angular.y, cur_state.twist.twist.angular.z;
  //   Eigen::Quaterniond q;
  //   tf::quaternionMsgToEigen(cur_state.pose.pose.orientation, q);
  //   R = q.toRotationMatrix();

  //   // Rotate omega
  //   omega = R.transpose() * omega;
  // }

  void onPidCmd(const ackermann_msgs::AckermannDriveStamped::ConstPtr& pid_cmd) {
    ackermann_speed = pid_cmd->drive.speed;
    ackermann_steering_angle = pid_cmd->drive.steering_angle;
  }

  void controlLoop(const ros::TimerEvent& t) {
    mav_msgs::Actuators msg;

    msg.angular_velocities.resize(4);
    msg.angular_velocities[0] = ackermann_speed;          // Acceleration
    msg.angular_velocities[1] = ackermann_steering_angle; // Turning angle
    msg.angular_velocities[2] = 0;                        // Breaking
    msg.angular_velocities[3] = 0;

    car_commands_pub.publish(msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
