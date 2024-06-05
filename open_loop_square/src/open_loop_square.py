#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import Range

class Duckie:
    def __init__(self):
        # Initialize variables
        self.cmd_msg = Twist2DStamped()
        self.tick_count = 0
        self.front_dis = 0

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        # Publishers and Subscribers
        self.pub = rospy.Publisher('/duckie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/duckie/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/duckie/left_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)
        rospy.Subscriber('/duckie/front_center_tof_driver_node/range', Range, self.range_callback, queue_size=1)

    def fsm_callback(self, msg):
        rospy.loginfo("Current Mode: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.halt_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)
            self.execute_square()

    def encoder_callback(self, msg):
        self.tick_count = msg.data

    def range_callback(self, msg):
        self.front_dis = msg.range

    def move_forward(self, distance, linear_speed):
        init_tick = self.tick_count
        while abs(self.tick_count - init_tick) < (distance * 100): 
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = linear_speed
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Moving forward!")
        self.halt_robot()

    def turn_left(self, angle, angular_speed):
        init_tick = self.tick_count
        while abs(self.tick_count - init_tick) < (angle * 100): 
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = angular_speed
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Turning left!")
        self.halt_robot()

    def halt_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def run(self):
        rospy.spin()

    def execute_square(self):
        for _ in range(4):
            self.move_forward(1.0, 0.5)  # Move forward 1 meter at 0.2 m/s
            self.turn_left(90, 1.0)      # Turn left 90 degrees at 1.0 rad/s
        self.halt_robot()

if __name__ == '__main__':
    try:
        duckie = Duckie()
        duckie.run()
    except rospy.ROSInterruptException:
        pass
