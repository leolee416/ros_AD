/*
 *  Traffic Lights Detection Node
 *
 *  This node is responsible for detecting traffic lights using data from
 *  semantic and RGB cameras. It subscribes to camera image topics, processes
 *  the images to identify and locate traffic lights, and determines their state.
 *  The node publishes the detected traffic light state and annotated images with
 *  bounding boxes around the detected lights.
 *
 *  Authors: Zhenjiang Li, Zhenyu Jin
 *  Date: 2024-07-20
 */

#include "PerceptionDetector.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <PerceptionDetector.h>
#include <sstream>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trafficlights_detector");

  ros::NodeHandle n;

  PerceptionDetector perception_detector(n);

  // ros::Rate loop_rate(10);

  perception_detector.subSemCam();
  perception_detector.subRGBCam();
  perception_detector.subBBox();
  perception_detector.subBoundaries();
  perception_detector.subDrawBBox();

  ros::spin();

  return 0;
}