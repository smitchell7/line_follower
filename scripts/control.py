#!/usr/bin/python

import rospy
from std_msgs.msg import Float32


class Control:
    def __init__(self):
        # Set up publishers and subscribers
        self.curvature_pub = rospy.Publisher(
            '/curvature_command', Float32, queue_size=2)
        self.vel_pub = rospy.Publisher(
            '/velocity_command', Float32, queue_size=2)
        rospy.init_node('control', anonymous=True)

        # Choose the method to control the vehicle
        control_method = 'circle'
        if control_method == 'circle':
            self.circle_control()
        elif control_method == 'fixed':
            rospy.Subscriber('/line_position', Float32,
                             self.fixed_number_control)
        elif control_method == 'proportional':
            rospy.Subscriber('/line_position', Float32,
                             self.proporional_control)

    def publish_msgs(self, vel_command, curv_command):
        self.curvature_pub.publish(curv_command)
        self.vel_pub.publish(vel_command)

    def circle_control(self):
        # Can we make the vehicle go faster or slower in a straight line?
        # Can we make the vehicle turn left or right?
        # How sharp can the vehicle turn?
        vel_command = 0.2
        curv_command = -1.0

        while not rospy.is_shutdown():
            self.publish_msgs(vel_command, curv_command)
            rospy.sleep(0.1)

    def fixed_number_control(self, msg):
        # Every time the line is measured, it sends a message with how far to
        # the side the detected line is. We can use that information to make
        # the robot react to the line.
        line_center = msg.data

        # Set up a way to turn if the vehicle is off the path.
        # Use 3 types of reactions:
        #   1. The line is centered.
        #   2. The line is to the right.
        #   3. The line is to the left.

        # How far off does the line need to be in order to react?
        line_threshold = 0.1
        go_straight = 0.0

        if abs(line_center) < line_threshold:
            curv_command = go_straight

        elif line_center > line_threshold:
            # Change this value to make the turtlebot respond
            curv_command = 0.0

        elif line_center < -1 * line_threshold:
            # Change this value to make the turtlebot respond
            curv_command = 0.0

        vel_command = 0.1
        self.publish_msgs(vel_command, curv_command)

    def proportional_control(self, msg):
        # Turning by a fixed magnitude makes the control jittery.
        # We can do better.

        # Right now, we turn left or right by a fixed amount. Instead we could
        # turn left a little bit if the line is a little right, and a lot left
        # if the line is very right.

        # Come up with an equation that would allow for smoother turn control
        # of the robot.
        # Hint. The equation might look like y=m*x+b.

        line_center = msg.data
        vel_command = 0.1
        curv_command = 0.0 * line_center

        self.publish_msgs(vel_command, curv_command)


if __name__ == '__main__':
    c = Control()
    rospy.spin()
