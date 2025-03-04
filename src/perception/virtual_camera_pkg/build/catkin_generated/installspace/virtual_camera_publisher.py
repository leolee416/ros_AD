#!/usr/bin/env python3

import rospy
import tf
from tf.transformations import quaternion_from_euler

def publish_virtual_camera():
    # Initialize the ROS node named 'virtual_camera_broadcaster'
    rospy.init_node('virtual_camera_broadcaster')
    
    # Create a TransformBroadcaster object to publish transformations
    br = tf.TransformBroadcaster()
    
    # Set the rate of publishing at 10 Hz
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        # Define the translation for the virtual camera relative to the real camera
        # In this case, the virtual camera is 3 meters behind and 1.5 meters above the real camera
        trans = (-3.0, 0.0, 1.5)
        
        # Define the rotation for the virtual camera
        # No additional rotation is applied, keeping the virtual camera aligned with the real camera
        rot = quaternion_from_euler(0, 0, 0)
        
        # Publish the transform with the current time
        br.sendTransform(trans,
                         rot,
                         rospy.Time.now(),
                         "virtual_camera",  # Child frame ID
                         "camera_link")     # Parent frame ID
        
        # Sleep for the remaining time to maintain the loop rate of 10 Hz
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_virtual_camera()
    except rospy.ROSInterruptException:
        pass
