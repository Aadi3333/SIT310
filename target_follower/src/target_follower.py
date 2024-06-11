#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        # Initialize publisher and subscriber with your robot's name
        self.cmd_vel_pub = rospy.Publisher('/duckie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/duckie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        # State of the robot: 'seeking' or 'tracking'
        self.state = 'seeking'
        
        # Control loop frequency
        self.rate = rospy.Rate(10)  # 10 Hz

        # Variables to store the detection info
        self.detection = None

        # Start control loop
        self.control_loop()

    # Control loop to send commands continuously
    def control_loop(self):
        while not rospy.is_shutdown():
            self.move_robot()
            self.rate.sleep()

    # April Tag Detection Callback
    def tag_callback(self, msg):
        self.detection = msg.detections[0] if msg.detections else None

    # Stop Robot before node has shut down. This ensures the robot doesn't keep moving with the last velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self):
        if not self.detection:
            self.state = 'seeking'
        else:
            self.state = 'tracking'
            x = self.detection.transform.translation.x
            z = self.detection.transform.translation.z
            rospy.loginfo("x, z: %f, %f", x, z)

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        if self.state == 'seeking':
            # Rotate in place to find a tag
            cmd_msg.v = 0.0
            cmd_msg.omega = 5.0  # Adjust this value to change rotation speed
        elif self.state == 'tracking':
            # Move forward/backward based on z distance
            if z > 0.15:
                cmd_msg.v = 0.2
                cmd_msg.omega = 0.0
            elif z < 0.10:
                cmd_msg.v = -0.2
                cmd_msg.omega = 0.0
            else:
                cmd_msg.v = 0.0
                cmd_msg.omega = 0.0

            # Rotate in place to keep the tag centered
            if x > 0.05:
                cmd_msg.omega = -4.4
            elif x < -0.05:
                cmd_msg.omega = 4.4
            else:
                cmd_msg.omega = 0.0

        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass

