#!/usr/bin/env python3

"""
Yolov5 ROS Node for Traffic Light Detection
This ROS node uses a pre-trained YOLOv5 model to detect traffic lights in images and 
publishes the bounding boxes and traffic light states. The detected traffic lights are
highlighted with corresponding colors in the output image.

Topics Subscribed:
- input_image_topic: The topic to subscribe to for input images.

Topics Published:
- output_topic: The topic to publish the bounding boxes of detected traffic lights.
- output_traffic_state_topic: The topic to publish the state of detected traffic lights.
- output_image_topic: The topic to publish the output images with detected traffic lights (optional).

Parameters:
- confidence_threshold: The confidence threshold for detection.
- iou_threshold: The Intersection over Union (IoU) threshold for non-max suppression.
- agnostic_nms: Whether to use class-agnostic non-max suppression.
- maximum_detections: The maximum number of detections.
- classes: The classes to detect.
- line_thickness: The thickness of bounding box lines.
- view_image: Whether to display the image with detections.
- weights: The path to the weights file of the YOLOv5 model.
- device: The device to run the model on (e.g., 'cpu' or 'cuda').
- dnn: Whether to use OpenCV DNN module.
- data: The path to the data configuration file.
- inference_size_w: The width of the input image for inference.
- inference_size_h: The height of the input image for inference.
- half: Whether to use half-precision inference.
- publish_image: Whether to publish the image with detections.
"""
import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type

from sensor_msgs.msg import Image
from perception_msgs.msg import BoundingBox, BoundingBoxes, TrafficState

# Add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolo_dependence"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# Import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox


@torch.no_grad()
class TrafficLightDetector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        # Initialize weights 
        weights = rospy.get_param("~weights")
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device","")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # '''
        # Logging model details for debugging and verification purposes
        # '''
        # rospy.loginfo('The value of stride is: %d', self.model.stride)
        # rospy.loginfo('The value of names is: %s', self.model.names)
        # rospy.loginfo('The value of pt is: %s', self.model.pt)
        # rospy.loginfo('The value of jit is: %s', self.model.jit)
        # rospy.loginfo('The value of onnx is: %s', self.model.onnx)
        # rospy.loginfo('The value of engine is: %s', self.model.engine)

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half precision setting
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        # Warmup model by running inference once
        self.model.warmup()         
        
        # Initialize subscriber to Image topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking=True)
        self.image_sub = rospy.Subscriber(input_image_topic, Image, self.image_callback, queue_size=1)

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic"), BoundingBoxes, queue_size=1
        )

        # Initialize traffic state publisher
        self.traffic_state_pub = rospy.Publisher(
            rospy.get_param("~output_traffic_state_topic"), TrafficState, queue_size=1
        )

        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=1
            )
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def image_callback(self, data):
        """
        Callback function of subscriber
        
        Output: 
        Three topics are published:
        1. /perception/detections (perception_msgs/BoundingBoxes):
            contains the bounding boxes of the detected objects
        2. /perception/traffic_state (perception_msgs/TrafficState): 
            contains the traffic state of the detected objects
        3. /perception/boundingbox_image(sensor_msgs/Image):
            contains the image with bounding boxes
        """
        im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        im, im0 = self.preprocess_image(im)

        # Run inference
        im = torch.from_numpy(im).to(self.device) 
        im = im.half() if self.half else im.float()
        im /= 255

        # Add a dimension if the dimension of the image equals 3, which means it doesn't have batch
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        )

        # Process predictions 
        det = pred[0].cpu().numpy()

        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = data.header
        bounding_boxes.image_header = data.header
        
        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))

        detected_red_light = False

        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det): # the bounding box with higher confidence will be processed first
                bounding_box = BoundingBox()
                traffic_state = TrafficState()
                c = int(cls)
                # Fill in bounding box message
                bounding_box.Class = self.names[c]
                bounding_box.probability = conf 
                bounding_box.xmin = int(xyxy[0])
                bounding_box.ymin = int(xyxy[1])
                bounding_box.xmax = int(xyxy[2])
                bounding_box.ymax = int(xyxy[3])

                # Fill in traffic state message
                # Green: 0, Red: 1, Yellow: 2
                if self.names[c] == "Red":
                    traffic_state.state = True
                    detected_red_light = True

                # Publish traffic state
                # The range is 0.4 to 0.6 of the image width 
                # Only one traffic light in the middle is used to sentence the traffic state
                if bounding_box.xmin > 0.4 * im0.shape[1] and bounding_box.xmax < 0.6 * im0.shape[1]:
                    self.traffic_state_pub.publish(traffic_state)

                # The range is 0.3 to 0.7 of the image width and the range is 0 to 0.5 of the image height
                # Multiple bounding boxes can be published at the same time.
                if bounding_box.xmin > 0.3 * im0.shape[1] and bounding_box.xmax < 0.7 * im0.shape[1] and bounding_box.ymax < 0.5 * im0.shape[0]:
                    bounding_boxes.bounding_boxes.append(bounding_box)

                # Annotate the image
                if self.publish_image or self.view_image:  # Add bbox to image
                    # integer class
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))       

            # Stream results
            im0 = annotator.result()

        # Publish default traffic state as false if no red light is detected
        if not detected_red_light:
            traffic_state = TrafficState()
            traffic_state.state = False
            self.traffic_state_pub.publish(traffic_state)

        # Publish prediction
        self.pred_pub.publish(bounding_boxes)

        # Publish & visualize images
        if self.view_image:
            cv2.imshow(str(0), im0)
            cv2.waitKey(1)  # 1 millisecond
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))
        

    def preprocess_image(self, img):
        """
        Preprocess the image for inference.
        
        Parameters:
        - img: The input image to preprocess.
        
        Returns:
        - A tuple containing the preprocessed image and the original image.
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 


if __name__ == "__main__":

    check_requirements(exclude=("tensorboard", "thop"))
    
    rospy.init_node("yolov5", anonymous=True)
    detector = TrafficLightDetector()
    
    rospy.spin()
