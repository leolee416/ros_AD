/*
 *  PID Controller Node
 *
 *  This node implements a PID controller for a Car. It subscribes to
 *  the /cmd_vel topic of move_base to receive desired velocity and steering commands, and to the
 *  /odom topic to receive odometry data. It uses PID control algorithms to compute
 *  the necessary control signals to achieve the desired velocity and steering angles.
 *  The computed control signals are published as AckermannDriveStamped messages
 *  to the /pid_cmd topic.
 *
 *  Author: Jincheng Pan
 *  Date: 2024-07-11
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <fstream>
#include <csignal>
#include <unistd.h> // For getcwd

class PidController
{
public:
  // Constructor
  PidController()
    : nh_()
    , kp_(1.76)
    , ki_(0.00)
    , kd_(0.01)
    , kp_yaw_(1.74)
    , ki_yaw_(0.00)
    , kd_yaw_(0.01)
    , integral_error_(0.0)
    , previous_error_(0.0)
    , integral_error_yaw_(0.0)
    , previous_error_yaw_(0.0)
    , wheelbase_(3.0)
    , desired_speed_(0.0)
    , desired_steering_angle_(0.0)
    , control_signal_(0.0)
    , steering_control_signal_(0.0)
    , derivative_error_(0.0)
    , derivative_error_yaw_(0.0)
  {
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &PidController::cmdVelCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &PidController::odomCallback, this);
    ackermann_cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/pid_cmd", 1);

    ROS_INFO(
        "Node 'cmd_vel_to_ackermann_drive' started.\nListening to /cmd_vel, publishing to /pid_cmd. Frame id: odom, "
        "wheelbase: %f",
        wheelbase_);
  }

  // Destructor
  ~PidController()
  {
    saveData();
  }

  // Function to save data
  void saveData()
  {
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != nullptr) {
        ROS_INFO("Current working directory: %s", cwd);
    } else {
        ROS_ERROR("getcwd() error");
    }

    std::ofstream file("error_data.txt");
    if (!file.is_open())
    {
      ROS_ERROR("Failed to open file error_data.txt");
      return;
    }

    ROS_INFO("Saving error data to error_data.txt");
    for (size_t i = 0; i < speed_errors_.size(); ++i)
    {
      file << ros::Time::now() << " " << speed_errors_[i] << " " << steering_errors_[i] << "\n";
    }
    file.close();
    ROS_INFO("Data saved successfully.");
  }

  // Main loop
  void spin()
  {
    ros::Rate rate(50);
    while (ros::ok())
    {
      publishAckermannCmd();  // Publish Ackermann command
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  // Callback for cmd_vel topic
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
  {
    double v = cmd_vel->linear.x;                                                            // Extract linear velocity
    double steering = convertTransRotVelToSteeringAngle(v, cmd_vel->angular.z, wheelbase_);  // Calculate steering angle

    desired_speed_ = v;                  // Update desired speed
    desired_steering_angle_ = steering;  // Update desired steering angle

    // ROS_INFO("Received /cmd_vel: linear.x = %f, angular.z = %f", v, cmd_vel->angular.z);
    // ROS_INFO("Computed steering angle: %f", steering);
  }

  // Callback for odometry topic
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
  {
    double current_speed = odom->twist.twist.linear.x;               // Current speed from odometry
    double current_steering_angle = getSteeringAngleFromOdom(odom);  // Current steering angle from odometry

    // PID control for speed
    double speed_error = desired_speed_ - current_speed;  // Calculate speed error
    integral_error_ += speed_error;                       // Integrate error
    derivative_error_ = speed_error - previous_error_;    // Calculate derivative of error
    control_signal_ = kp_ * speed_error + ki_ * integral_error_ + kd_ * derivative_error_;  // Compute control signal
    previous_error_ = speed_error;                                                          // Update previous error
    speed_errors_.push_back(speed_error);  // Record speed error

    // PID control for steering angle
    double steering_error = desired_steering_angle_ - current_steering_angle;  // Calculate steering error
    integral_error_yaw_ += steering_error;                                     // Integrate error
    derivative_error_yaw_ = steering_error - previous_error_yaw_;              // Calculate derivative of error
    steering_control_signal_ = kp_yaw_ * steering_error + ki_yaw_ * integral_error_yaw_ +
                               kd_yaw_ * derivative_error_yaw_;  // Compute control signal
    previous_error_yaw_ = steering_error;                        // Update previous error
    steering_errors_.push_back(steering_error);  // Record steering error

    // ROS_INFO("Received odom: current_speed = %f, current_steering_angle = %f", current_speed,
    // current_steering_angle); ROS_INFO("PID control: speed_control_signal = %f, steering_control_signal = %f",
    // control_signal_, steering_control_signal_);
  }

  // Publish Ackermann command
  void publishAckermannCmd()
  {
    ackermann_msgs::AckermannDriveStamped ackermann_msg;
    ackermann_msg.header.stamp = ros::Time::now();
    ackermann_msg.header.frame_id = "odom";
    ackermann_msg.drive.steering_angle = -steering_control_signal_;  // Publish steering control signal
    ackermann_msg.drive.speed = control_signal_;                     // Publish speed control signal

    ackermann_cmd_pub_.publish(ackermann_msg);  // Publish the message

    // ROS_INFO("Published /pid_cmd: steering_angle = %f, speed = %f", ackermann_msg.drive.steering_angle,
    // ackermann_msg.drive.speed);
  }

  // Calculate the steering angle from odometry
  double getSteeringAngleFromOdom(const nav_msgs::Odometry::ConstPtr& odom)
  {
    double yaw_rate = odom->twist.twist.angular.z;     // Angular velocity of the vehicle
    double linear_speed = odom->twist.twist.linear.x;  // Linear velocity of the vehicle
    if (linear_speed == 0.0)
      return 0.0;  // Avoid division by zero

    return atan2(yaw_rate * wheelbase_, linear_speed);  // Calculate and return steering angle
  }

  // Convert translational and rotational velocities to a steering angle
  double convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase)
  {
    if (omega == 0 || v == 0)
    {
      return 0;
    }
    double radius = v / omega;        // Calculate turning radius
    return atan(wheelbase / radius);  // Calculate and return steering angle
  }

  ros::NodeHandle nh_;                // ROS node handle
  ros::Subscriber cmd_vel_sub_;       // Subscriber for cmd_vel topic
  ros::Subscriber odom_sub_;          // Subscriber for odometry topic
  ros::Publisher ackermann_cmd_pub_;  // Publisher for Ackermann command topic

  std::string twist_cmd_topic_;      // Topic name for twist command
  std::string ackermann_cmd_topic_;  // Topic name for Ackermann command
  std::string frame_id_;             // Frame ID for the messages
  double wheelbase_;                 // Wheelbase of the vehicle

  double kp_, ki_, kd_;                             // PID gains for speed control
  double kp_yaw_, ki_yaw_, kd_yaw_;                 // PID gains for steering control
  double integral_error_, previous_error_;          // PID variables for speed control
  double integral_error_yaw_, previous_error_yaw_;  // PID variables for steering control
  double desired_speed_;                            // Desired speed
  double desired_steering_angle_;                   // Desired steering angle
  double control_signal_;                           // Control signal for speed
  double steering_control_signal_;                  // Control signal for steering
  double derivative_error_;                         // Derivative error for speed
  double derivative_error_yaw_;                     // Derivative error for steering

  std::vector<double> speed_errors_;                // Vector to record speed errors
  std::vector<double> steering_errors_;             // Vector to record steering errors
};

PidController* controller_ptr = nullptr;

void signalHandler(int signum)
{
  if (controller_ptr)
  {
    ROS_INFO("SIGINT caught, saving data...");
    controller_ptr->saveData();
  }
  ros::shutdown();
  exit(signum);
}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "pid_controller");
  // Create an instance of the PidController
  PidController controller;
  controller_ptr = &controller;

  // Set up signal handler
  signal(SIGINT, signalHandler);

  // Run the controller
  controller.spin();
  return 0;
}
