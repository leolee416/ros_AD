/*
 *  Perception Detector Node
 *
 *  This node implements a perception detector for a car. It subscribes to
 *  semantic camera and RGB camera topics to detect traffic lights and determine their state.
 *  It also processes image data to extract bounding boxes for detected objects and draws them on the images.
 *  The node publishes the traffic state and the modified images with bounding boxes.
 *
 *  Author: Zhenjiang Li, Zhenyu Jin
 *  Date: 2024-07-20
 */

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32MultiArray.h"
#include <cstdint>
#include <cstdlib>
#include <PerceptionDetector.h>
#include <sstream>
#include <vector>
#include "perception_msgs/TrafficState.h"

PerceptionDetector::PerceptionDetector(ros::NodeHandle &nh)
    : nh_(nh),
      pub_traffic_state_(nh.advertise<perception_msgs::TrafficState>("/perception/traffic_state", 1)),
      pub_BoundingBoxImage_(nh.advertise<sensor_msgs::Image>("/perception/boundingbox_image", 1))
{
  // Initialize the state of the traffic state message to false
  msg_traffic_state_.state = false;
}

void PerceptionDetector::semeticCallback(const sensor_msgs::ImageConstPtr &seg_img)
{
  /**
   * @brief Extract the coordinates of the traffic light in the semantic camera image.
   *
   * @param seg_img The input image message from the semantic camera.
   *
   * This function processes the input image to find pixels corresponding to traffic lights,
   * specifically looking for yellow pixels in the specified region of the image.
   * The coordinates of these pixels are stored in the area_trafficlights_ vector.
   */

  // Get the image data
  const std::vector<uint8_t> &img_vec = seg_img->data;

  // Get the width and height of the image
  img_width_ = seg_img->width;
  const int img_height = seg_img->height;

  // Define the region of interest (ROI) and color thresholds
  const int left_bound = 349;
  const int right_bound = 635;
  const int low_bound = img_height / 3;
  const uint8_t r_threshold = 255;
  const uint8_t g_threshold = 235;
  const uint8_t b_threshold = 4;

  // Loop through the ROI in the upper part of the image
  for (int i = 0; i < low_bound; ++i)
  {
    for (int j = left_bound; j < right_bound; j += 3)
    {
      // Calculate the index of the pixel in the image data
      int index = i * img_width_ * 3 + j;
      // Check green channel only
      if (img_vec[index] == g_threshold)
      {
        // Push the pixel index into the vector
        area_trafficlights_.push_back(index);
      }
      // // Check if the pixel matches the yellow color (255, 235, 4)
      // if (img_vec[index - 1] == r_threshold && img_vec[index] == g_threshold && img_vec[index + 1] == b_threshold)
      // {
      //   // Push the pixel index into the vector
      //   area_trafficlights_.push_back(index);
      // }
    }
  }
}

void PerceptionDetector::RGBCallback(const sensor_msgs::ImageConstPtr &rgb_img)
{
  /**
   * @brief Callback function to process the RGB image and determine the traffic light state.
   *
   * @param rgb_img The input RGB image message.
   *
   * This function checks the specified areas in the image to detect the color of the traffic light.
   * If the light is red, it sets the traffic state to stop. If the light is green, it sets the traffic state to go.
   */

  static ros::Time last_check_time = ros::Time::now(); // Static variable to store last check time
  ros::Duration check_interval(1.0);                   // 1 second interval

  const std::vector<uint8_t> &img_vec = rgb_img->data;
  bool state_changed = false; // Flag to check if state has changed
  bool found_light = false;   // Flag to check if any light is found

  // Define the color thresholds for red and green lights
  const uint8_t red_r_threshold = 180;
  const uint8_t red_g_threshold = 80;
  const uint8_t red_b_threshold = 80;

  const uint8_t green_r_threshold = 150;
  const uint8_t green_g_threshold = 180;
  const uint8_t green_b_threshold = 150;

  for (size_t i = 0; i < area_trafficlights_.size(); ++i)
  {
    int index = area_trafficlights_[i];
    // Ensure the index is within bounds
    if (index - 1 < 0 || index + 1 >= img_vec.size())
    {
      continue; // Skip if out of bounds
    }

    // Check for red light
    if (img_vec[index - 1] > red_r_threshold &&
        img_vec[index] < red_g_threshold &&
        img_vec[index + 1] < red_b_threshold)
    {
      // ROS_INFO_STREAM("Red Light!");
      // Set traffic state to false
      msg_traffic_state_.state = true;
      state_changed = true;
      found_light = true; // Mark that a light was found
      break;              // Exit the loop once a red light is found
    }
    // Check for green light
    else if (img_vec[index - 1] < green_r_threshold &&
             img_vec[index] > green_g_threshold &&
             img_vec[index + 1] < green_b_threshold)
    {
      // ROS_INFO_STREAM("Green Light!");
      // Set traffic state to true
      msg_traffic_state_.state = false;
      state_changed = true;
      found_light = true; // Mark that a light was found
      break;              // Exit the loop once a green light is found
    }
  }

  // Publish the traffic state immediately if it has changed
  if (state_changed)
  {
    pub_traffic_state_.publish(msg_traffic_state_);
  }

  // Check if it's time to update the default light state
  if (ros::Time::now() - last_check_time >= check_interval)
  {
    last_check_time = ros::Time::now(); // Update the last check time

    // If no light was found, default to green light
    if (!found_light)
    {
      // ROS_INFO_STREAM("No Red or Green Light found, defaulting to Green Light!");
      msg_traffic_state_.state = false;
      pub_traffic_state_.publish(msg_traffic_state_); // Publish the default green light state
    }
  }

  // Clear the traffic lights area after processing
  area_trafficlights_.clear();
}

void PerceptionDetector::getBoundingBoxCallback(const sensor_msgs::Image::ConstPtr &msg)
{
  /**
   * Description:
   * From semantic image data get a bounding box area of this image
   *
   * Input:
   * Image msg.
   *
   * Output:
   * The X and Y axis of the bounding box
   */

  const std::vector<uint8_t> &image_data = msg->data;
  int image_width = msg->width;
  int image_height = msg->height;

  // Variables to store the bounding box coordinates
  min_x_ = image_width; // Initialize with maximum possible value
  min_y_ = image_height;
  max_x_ = 0; // Initialize with minimum possible value
  max_y_ = 0;

  // Iterate over the pixels and analyze RGB values
  for (int y = 0; y < 120; ++y)
  {
    for (int x = 135; x < 185; ++x)
    {
      int pixel_index =
          (y * image_width + x) * 3; // Index of the pixel in the 1D vector

      // Extract RGB values of the pixel
      uint8_t red = image_data[pixel_index];
      uint8_t green = image_data[pixel_index + 1];
      uint8_t blue = image_data[pixel_index + 2];

      // Check if the pixel belongs to the segmented object
      // Here, you can define your own criteria based on RGB values
      if (red == 255 && green == 235 && blue == 4)
      {
        // Update the bounding box coordinates if necessary
        if (x < min_x_)
          min_x_ = x;
        if (x > max_x_)
          max_x_ = x;
        if (y < min_y_)
          min_y_ = y;
        if (y > max_y_)
          max_y_ = y;
      }
    }
  }
}

void PerceptionDetector::findMinMaxCoordinatesCallback(const sensor_msgs::Image::ConstPtr &msg)
{
  // Initialize min and max values
  min_x_ = std::numeric_limits<int>::max();
  max_x_ = std::numeric_limits<int>::min();
  min_y_ = std::numeric_limits<int>::max();
  max_y_ = std::numeric_limits<int>::min();

  for (const int index : area_trafficlights_)
  {
    // Calculate y coordinate
    int y = index / (img_width_ * 3);
    // Calculate x coordinate
    int x = (index % (img_width_ * 3)) / 3;

    // Update min and max values
    if (x < min_x_)
      min_x_ = x;
    if (x > max_x_)
      max_x_ = x;
    if (y < min_y_)
      min_y_ = y;
    if (y > max_y_)
      max_y_ = y;
  }
}

void PerceptionDetector::drawLine(sensor_msgs::Image &image, int start, int end, int constant, bool horizontal, uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = start; i <= end; ++i)
  {
    int index;
    if (horizontal)
    {
      index = (constant * image.width + i) * 3;
    }
    else
    {
      index = (i * image.width + constant) * 3;
    }
    image.data[index] = r;     // Red channel
    image.data[index + 1] = g; // Green channel
    image.data[index + 2] = b; // Blue channel
  }
}

void PerceptionDetector::drawBoundingBoxCallback(const sensor_msgs::Image::ConstPtr &original_msg)
{
  /**
   * Description:
   * Draw the bounding box on an RGB image based on the traffic state.
   *
   * Input:
   * - original_msg: The original image message containing the RGB image.
   *
   * Output:
   * - Publishes the modified image message with a bounding box drawn on it.
   */

  // Create a new image message for the modified image
  sensor_msgs::Image modified_msg;
  modified_msg.header = original_msg->header;
  modified_msg.height = original_msg->height;
  modified_msg.width = original_msg->width;
  modified_msg.encoding = original_msg->encoding;
  modified_msg.is_bigendian = original_msg->is_bigendian;
  modified_msg.step = original_msg->step;
  modified_msg.data = original_msg->data;

  // Determine the color of the bounding box based on the traffic state
  uint8_t r, g, b;
  if (!msg_traffic_state_.state)
  {
    r = 0;
    g = 255;
    b = 0;
  }
  else
  {
    r = 255;
    g = 0;
    b = 0;
  }

  // Draw the bounding box on the image
  drawLine(modified_msg, min_x_, max_x_, min_y_, true, r, g, b);  // Top line
  drawLine(modified_msg, min_x_, max_x_, max_y_, true, r, g, b);  // Bottom line
  drawLine(modified_msg, min_y_, max_y_, min_x_, false, r, g, b); // Left line
  drawLine(modified_msg, min_y_, max_y_, max_x_, false, r, g, b); // Right line

  // Publish the modified image
  pub_BoundingBoxImage_.publish(modified_msg);
}

void PerceptionDetector::subSemCam()
{
  sub_sem_cam_ =
      nh_.subscribe("/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 1,
                    &PerceptionDetector::semeticCallback, this);
}

void PerceptionDetector::subRGBCam()
{
  sub_RGB_cam_ = nh_.subscribe("/unity_ros/OurCar/Sensors/RGBCameraLeft/image_raw", 1,
                               &PerceptionDetector::RGBCallback, this);
}

void PerceptionDetector::subBBox()
{
  sub_getBoundingbox_ =
      nh_.subscribe("/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 1,
                    &PerceptionDetector::getBoundingBoxCallback, this);
}

void PerceptionDetector::subBoundaries()
{

  sub_getBoundingbox_ =
      nh_.subscribe("/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 1,
                    &PerceptionDetector::findMinMaxCoordinatesCallback, this);
}

void PerceptionDetector::subDrawBBox()
{
  sub_drawBoundingBox_ =
      nh_.subscribe("/unity_ros/OurCar/Sensors/RGBCameraLeft/image_raw", 1,
                    &PerceptionDetector::drawBoundingBoxCallback, this);
}
