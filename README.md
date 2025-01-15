# TUM Project Introduction to ROS: Autonomous Driving
This is the repo for the group project on autonomous driving of Team 15 for the lecture **Introduction to ROS** in the summer semester 2024.

# Project Overview
*(The objective of this project is to achieve automatic driving of a vehicle on a specified route. The vehicle must complete the entire route in the shortest possible time while avoiding obstacles and complying with traffic rules. We have divided the project into three main components: perception, planning, and control. These components are interconnected through a state machine.
In the perception component, we employ two methods: a traditional computer vision (CV) approach and a learning-based approach. For the planning component, we utilize the way\_point\_global planner for global planning and the Teb Planner for local planning. Finally, in the control component, we use a PID controller for vehicle control.)*
## Team Members

### Zhenjiang Li
- **Area of Responsibility**: Perception
- **Responsibilities**:Pointcloud, octomap,NN detector,Traffic Light Localization,  Recognition and Box Drawing.
- **Contact**: [zhenjiang1.li@tum.de](zhenjiang1.li@tum.de)
### Zhenyu Jin
- **Area of Responsibility**: Perception
- **Responsibilities**: NN detector, Traffic Light Localization, Color Recognition and Box Drawing
- **Contact**: [go98zid@mytum.com](go98zid@mytum.com)
### Jincehng Pan
- **Area of Responsibility**: Planning, Controller
- **Responsibilities**: palnning nodes implementation, `move_base` configuration and implementation, PID controller implementation,
- **Contact**: [ge53huy@mytum.com](ge53huy@mytum.com)
### Rongxin Wang
- **Area of Responsibility**: Planning, Controller
- **Responsibilities**: `move_base` and PID controller tuning, waypoint setting and parameter adjustment
- **Contact**: [ge53bik@mytum.com](ge53bik@mytum.com)
### Chen Yu
- **Area of Responsibility**: State Machine
- **Responsibilities**: State machine writing, code integration
- **Contact**: [go69wig@mytum.com](mailto:go69wig@mytum.com)

# Contents
- [TUM Project Introduction to ROS: Autonomous Driving](#tum-project-introduction-to-ros-autonomous-driving)
- [Project Overview](#project-overview)
  - [Team Members](#team-members)
    - [Zhenjiang Li](#zhenjiang-li)
    - [Zhenyu Jin](#zhenyu-jin)
    - [Jincehng Pan](#jincehng-pan)
    - [Rongxin Wang](#rongxin-wang)
    - [Chen Yu](#chen-yu)
- [Contents](#contents)
  - [Result](#result)
    - [Video](#video)
    - [Image](#image)
      - [Global map](#global-map)
    - [Rosgraph](#rosgraph)
    - [TF tree](#tf-tree)
    - [Realized Tasks](#realized-tasks)
- [Required Dependencies](#required-dependencies)
- [Quick Start](#quick-start)
- [Modules description](#modules-description)
  - [Perception](#perception)
  - [Planning](#planning)
  - [Control](#control)


## Result

### Video
Demo_video: [Watch the demo video](https://gitlab.lrz.de/i2ros_group_15/autonomous_driving/-/raw/main/video/Demo_video.mp4?ref_type=heads&inline=false)
### Image
#### Global map
![Global map](https://gitlab.lrz.de/i2ros_group_15/autonomous_driving/-/raw/main/image/Screenshot%20from%202024-07-29%2022-07-44.png?ref_type=heads&inline=false)
### Rosgraph
![Rosgraph](https://gitlab.lrz.de/i2ros_group_15/autonomous_driving/-/raw/main/image/rosgraph.png?ref_type=heads&inline=false)
### TF tree
![TF Tree](https://gitlab.lrz.de/i2ros_group_15/autonomous_driving/-/raw/main/image/frames.png?ref_type=heads&inline=false)

### Realized Tasks
- [x] Successfully working perception pipeline 
- [x] Successfully working path planning
- [x] Successfully working trajectory planning 
- [x] Successfully avoiding other cars
- [x] Successfully stopping/driving at street lights 
- [x] Time to complete the mission: 189s(semantic solution), 225s(DL solution)
- [x] Solving the problem without using semantic camera: YOLOv5
- [x] New messages: `perception_msgs/Boundingboxes.msg`, `perception_msgs/Boundingbox.msg`, `perception_msgs/Trafficstate.msg`
- [x] New service: `planning/srv/Waypoint.srv`

# Required Dependencies

- "Due to the high control frequency, generating outputs within the specified time frame places significant demands on computer performance; otherwise, it may deviate from the control cycle."

- Run the sh file to install all- [Prerequisites](#prerequisites) packages

```shell
chmod +x requirements.sh
./dependency/dependency_requirements.sh group
```

- Or Run following command to install required packages.

Perception:
```shell
sudo apt install ros-noetic-octomap-rviz-plugins ros-noetic-octomap-server
pip install -r Autonomous_Driving_ws/src/perception/yolov5/src/yolov5/requirements.txt
```

Control:
```shell
sudo apt install ros-noetic-pid ros-noetic-robot-localization ros-noetic-smach-ros
sudo apt-get install ros-noetic-ackermann-msgs
```

Planning:
```shell
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-teb-local-planner
```
**Tip:** If some dependencies for YOLO or CUDA are not installed, you can install them manually via “autonomous_driving/src/perception/nn_perception_pkg/src/yolo_dependence/requirements. txt” to install the dependencies manually.

# Quick Start
1. Build it with `catkin build`.
2. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiLvgiTXetubiN1i4PRjuR/
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. source the devel to get access to the ros nodes and topics.
```shell
source devel/setup.bash // for bash shell user
```
or
```shell
source devel/setup.zsh // for zsh shell user
```
5. run the following command to launch version with 
```shell
roslaunch simulation nn_simulation.launch 
```
or run the following command to launch version with Semantic Camera
```shell
roslaunch simulation sem_simulation.launch 
```
  
The car will start driving along the generated global and local path.
The traffic rules will be followed correctly.

**Hint:** If traffic_light_detector_node died with error, please rerun the launch file again. It may occur due to conflict in roslog file. In this case, you can try cleaning the roslog files and then run `catkin clean` and `catkin build` again.

# Modules description
## Perception
- perception_pkg: including a node ```trafficlights_detect_node```, which extract the area of traffic light from semantic image and then recognize the color of the traffic light in RGB image. It gives the controller the state of traffic light to stop the car of let it move again. Additionally a bounding box in color red or green is also drawn and outputed by this node.
- depth_image_proc: a package to convert the depth image data to 3D point cloud data.
- OctoMap: a package which can get the occupancy grids and map from 3D point cloud data.
- yolov5: A package to perform traffic light detection using YOLOv5. It includes a ```cnn_detect``` ros node, which uses the trained YOLOv5 model for inference and publish the detection results.

## Planning
- we use `move_base` as the primary package
- **planning**: node `waypoint_server` sends waypoints for waypoint global planner; node `global_path_planner` considers traffic lights and car position and decides when and which waypoints will be sent.
- move_base: primary pakage used for planning and navigation Tasks
- global planner: waypoint_global_planner plugin(opensource from GitHub)
- local planner: teb planner

## Control
- **controller_pkg**: The `pid_controller` node converts `cmd_vel` to Ackermann messages and uses a PID algorithm to calculate the final velocity command values. The `controller_node` receives the final velocity command and sends it to the Unity environment.
