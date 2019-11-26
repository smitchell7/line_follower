#!/usr/bin/python

import rospy
from std_msgs.msg import Float32

class Control:
    def __init__(self):
        # Set up publishers and subscribers
        self.curvature_pub = rospy.Publisher('/curvature_command', Float32, queue_size=2)
        self.vel_pub = rospy.Publisher('/velocity_command', Float32, queue_size=2)
        rospy.init_node('control', anonymous=True)

        # Choose the method to control the vehicle
        control_method = 'fixed'
        if control_method == 'fixed':
            rospy.Subscriber('/line_position', Float32, self.fixed_number_control)
        elif control_method == 'proportional':
            rospy.Subscriber('/line_position', Float32, self.proporional_control)

    def fixed_number_control(self, msg):
        line_center = msg.data
        vel_command = 0.2

        line_threshold = 0.030
        turn_magnitude = 0.7
        go_straight = 0.0

        # use line center to choose to turn left, right, or go straight.

        curv_command = go_straight

        # # TODO remove this.
        # if line_center > line_threshold:
        #     # turn right if line is to the left
        #     curv_command = -1 * turn_magnitude

        # elif line_center < -1 * line_threshold:
        #     # turn left if the line is to the right
        #     curv_command = turn_magnitude

        # else:
        #     curv_command = go_straight

        self.curvature_pub.publish(curv_command)
        self.vel_pub.publish(vel_command)

    def proportional_control(self,msg):
        line_center = msg.data
        vel_command = 0.2
        curv_command = 0.0

        # how would we calculate a curvature command using a proportional
        # equation (ratio)?

        self.curvature_pub.publish(curv_command)
        self.vel_pub.publish(vel_command)


if __name__ == '__main__':
    c = Control()
    rospy.spin()



