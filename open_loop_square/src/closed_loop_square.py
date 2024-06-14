#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import Range
import math
 
class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()
        self.tick_count = 0
        self.front_dis = 0
        self.start_tick = 0  # To store initial tick count
        
        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)
        
        # Initialize Pub/Subs
        self.pub = rospy.Publisher('/duckie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/duckie/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/duckie/left_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)
        rospy.Subscriber('/duckie/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)
        
    def fsm_callback(self, msg):
        # Callback function for FSM state changes
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Wait for a sec for the node to be ready
            self.move_robot()
    
    def encoder_callback(self, msg):
        # Callback function for wheel encoder data
        self.tick_count = msg.data
        if self.start_tick == 0:
            self.start_tick = self.tick_count
    
    def range_callback(self, msg):
        # Callback function for range sensor data
        self.front_dis = msg.range
    
    def goal_distance(self, distance, linear_speed):
        # Move forward for a specified distance at given speed
        
        initial_tick = self.tick_count
        while abs(self.tick_count - initial_tick) < (distance * 200):
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = linear_speed
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Forward!")
            rospy.sleep(0.7)  # Adjust sleep time as needed
        self.stop_robot()

    def goal_angle(self, angle, angular_speed):
        # Rotate the robot by a specified angle at given angular speed
        
        initial_tick = self.tick_count
        while abs(self.tick_count - initial_tick) < (angle * 25): 
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = angular_speed
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Rotate!")
            rospy.sleep(0.7)  # Adjust sleep time as needed
        self.stop_robot()
    
    def rotate_in_place_360(self, angular_speed):
        # Rotate the robot 360 degrees (2 * pi radians) in place
        
        initial_tick = self.tick_count
        target_angle = 2 * math.pi  # 360 degrees in radians
        while abs(self.tick_count - initial_tick) < (target_angle * 25): 
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = angular_speed
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Rotate 360!")
            rospy.sleep(0.7)  # Adjust sleep time as needed
        self.stop_robot()
 
    def stop_robot(self):
        # Sends zero velocities to stop the robot
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
 
    def run(self):
        # Spin forever but listen to message callbacks
        rospy.spin()

    def move_robot(self):
        # Robot drives in a square pattern and then stops
        
        for _ in range(4):
            self.goal_distance(1, 0.5)  # Move forward 1 meter at 0.5 m/s
            self.goal_angle(1.57, 1.0)  # Rotate approximately 90 degrees at 1 rad/s
        
        # After completing the square, rotate 360 degrees in place
        self.rotate_in_place_360(1.0)  # Rotate 360 degrees at 1 rad/s
        
        self.stop_robot()  # Stop the robot after completing the square and rotation

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass