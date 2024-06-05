#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class SquareDriver:
    def __init__(self):
        # Initialize variables
        self.motion_cmd = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('square_driver_node', anonymous=True)

        # Publishers and Subscribers
        self.command_pub = rospy.Publisher('/duckie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/duckie/fsm_node/mode', FSMState, self.mode_callback, queue_size=1)

    def mode_callback(self, msg):
        rospy.loginfo("Current Mode: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.halt_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)
            self.execute_square()

    def halt_robot(self):
        self.motion_cmd.header.stamp = rospy.Time.now()
        self.motion_cmd.v = 0.0
        self.motion_cmd.omega = 0.0
        self.command_pub.publish(self.motion_cmd)

    def run(self):
        rospy.spin()

    def execute_square(self):
        # Move and turn in a square pattern
        for _ in range(4):
            self.move_forward(1.0)
            self.turn_left(90)
        # Ensure the robot stops at the end
        self.halt_robot()

    def move_forward(self, meters):
        # Adjust duration to cover 1 meter; this is an approximate value
        speed = 0.2
        duration = meters / speed

        self.motion_cmd.header.stamp = rospy.Time.now()
        self.motion_cmd.v = speed
        self.motion_cmd.omega = 0.0
        self.command_pub.publish(self.motion_cmd)
        rospy.loginfo(f"Moving forward {meters} meters...")
        rospy.sleep(duration)

    def turn_left(self, degrees):
        # Adjust duration to turn 90 degrees; this is an approximate value
        angular_speed = 1.0
        duration = degrees / 90.0  # Assume it takes 1 second to turn 90 degrees at 1.0 rad/s

        self.motion_cmd.header.stamp = rospy.Time.now()
        self.motion_cmd.v = 0.0
        self.motion_cmd.omega = angular_speed
        self.command_pub.publish(self.motion_cmd)
        rospy.loginfo(f"Turning left {degrees} degrees...")
        rospy.sleep(duration)
        self.halt_robot()

if __name__ == '__main__':
    try:
        driver = SquareDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass

