/*
 *  Perception Detector Class Definition
 *
 *  This header file defines the PerceptionDetector class, which is responsible for
 *  detecting traffic lights using data from semantic and RGB cameras. The class subscribes
 *  to camera image topics, processes the images to identify and locate traffic lights,
 *  determines their state, and publishes the detected traffic light state and annotated
 *  images with bounding boxes around the detected lights.
 *
 *  Authors: Zhenjiang Li, Zhenyu Jin
 *  Date: 2024-07-20
 */

#ifndef PERCEPTION_DETECTOR_H
#define PERCEPTION_DETECTOR_H

#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32MultiArray.h"
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <vector>
#include "perception_msgs/TrafficState.h"

class PerceptionDetector
{
public:
  PerceptionDetector(ros::NodeHandle &nh);
  void semeticCallback(const sensor_msgs::ImageConstPtr &seg_img);
  void RGBCallback(const sensor_msgs::ImageConstPtr &rgb_img);
  void getBoundingBoxCallback(const sensor_msgs::Image::ConstPtr &msg);
  void drawBoundingBoxCallback(const sensor_msgs::Image::ConstPtr &original_msg);
  void subSemCam();
  void subRGBCam();
  void findMinMaxCoordinatesCallback(const sensor_msgs::Image::ConstPtr &msg);
  void subBBox();
  void subBoundaries();
  void subDrawBBox();
  void drawLine(sensor_msgs::Image &image, int start, int end, int constant, bool horizontal, uint8_t r, uint8_t g, uint8_t b);

private:
  ros::Publisher pub_traffic_state_;
  ros::Publisher pub_BoundingBoxImage_;
  ros::Timer timer;

  ros::Subscriber sub_sem_cam_;
  ros::Subscriber sub_RGB_cam_;
  ros::Subscriber sub_getBoundingbox_;
  ros::Subscriber sub_drawBoundingBox_;
  ros::ServiceServer service;
  ros::NodeHandle &nh_;

  // std_msgs::Bool msg_traffic_state_;
  perception_msgs::TrafficState msg_traffic_state_;
  sensor_msgs::Image msg_RGB_cam_;
  std::vector<uint32_t> area_trafficlights_;

  int min_x_, max_x_, min_y_, max_y_;

  int img_width_;
};

#endif
