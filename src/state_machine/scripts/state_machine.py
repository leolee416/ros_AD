import rospy
import smach
import smach_ros
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist,Point


signal_value = False
target_twist = Twist()
start_time = None
timing_started = False
end_position = Point()
end_position.x = 5.00
end_position.y = -63.90


def signal_callback(signal_msg):
    global signal_value
    signal_value = signal_msg.data 

def position_callback(position_msg):
    global timing_started, start_time
    if timing_started:
        distance = ((position_msg.x - end_position.x) ** 2 + 
                    (position_msg.y - end_position.y) ** 2 ) ** 0.5
        if distance < 0.1:  # Assuming a small threshold to determine if arrived the goal position
            end_time = rospy.Time.now()
            travel_time = end_time - start_time
            rospy.loginfo(f"Total travel time: {travel_time.to_sec()} seconds")
            timing_started = False #stop timing

class Drive(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['stop'])
            self.target_v_pub = rospy.Publisher("target_linear_velocity", Float64, queue_size=10)
            self.target_omega_pub = rospy.Publisher("target_angular_velocity", Float64, queue_size=10)
       # Your state initialization goes here

        def execute(self, userdata):
            global start_time, timing_started
            rospy.loginfo("Executing state Drive")
            rate = rospy.Rate(100)
        # Your state execution goes here
            if not timing_started:
                start_time = rospy.Time.now()
                timing_started = True
                rospy.loginfo("Timing started")
            while not rospy.is_shutdown():
                if not signal_value:
                    self.target_v_pub.publish(Float64(target_twist.linear.x))
                    self.target_omega_pub.publish(Float64(target_twist.angular.z))
        
                rate.sleep()
            return 'stop'
        
class Stop(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['drive'])
            self.target_v_pub = rospy.Publisher("target_linear_velocity", Float64, queue_size=10)
            self.target_omega_pub = rospy.Publisher("target_angular_velocity", Float64, queue_size=10)
       # Your state initialization goes here

        def execute(self, userdata):
            rospy.loginfo("Executing state Stop")
            rate = rospy.Rate(100)
        # Your state execution goes here
            while not rospy.is_shutdown():
                if signal_value:
           
                    self.target_v_pub.publish(Float64(0.0))
                    self.target_omega_pub.publish(Float64(0.0))    
                rate.sleep()
            return 'drive'
    

def twist_callback(target_twist_msg):
    global target_twist
    target_twist = target_twist_msg


def main():
    rospy.init_node("smach")

    rospy.Subscriber('signal_topic', Bool, signal_callback)
    rospy.Subscriber('target_twist', Twist, twist_callback)



    
    
    
    sm = smach.StateMachine(outcomes=[])
    with sm:
        smach.StateMachine.add('DRIVE', Drive(),
                            transitions={'stop':'STOP'})
        smach.StateMachine.add('STOP', Stop(),
                           transitions={'drive':'DRIVE'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

if __name__ == '__main__':
    main()
