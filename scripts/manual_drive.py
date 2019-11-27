#!/usr/bin/python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class Steering:

    def __init__(self):
        self.curvature_pub = rospy.Publisher(
            '/curvature_command', Float32, queue_size=2)
        self.vel_pub = rospy.Publisher(
            '/velocity_command', Float32, queue_size=2)
        rospy.init_node('control', anonymous=True)
        self.gear = 0
        self.steer_command = 0
        self.vel_command = 0

        rospy.Subscriber('/joy', Joy, self.steering_cb)

    def publish_msgs(self, vel_command, curv_command):
        self.curvature_pub.publish(curv_command)
        self.vel_pub.publish(vel_command)

    def get_gear(self, buttons):
        fwd = sum(buttons[0::2])
        rev = sum(buttons[1::2])
        gear = fwd - 1 * rev
        if gear != self.gear:
            rospy.loginfo("gear: {}".format(gear))
            self.gear = gear
        return gear


    def steering_cb(self, msg):
        steer_input = msg.axes[0] 
        pedal_input = ( msg.axes[2] + 1 ) / 2
        gear_input = self.get_gear(msg.buttons[12:18])

        sign = lambda a: (a>0) - (a<0)
        steer_input = sign(steer_input) * ( (steer_input * 2) ** 2 )

        self.steer_command = steer_input
        self.vel_command = pedal_input * gear_input

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.publish_msgs(self.vel_command, self.steer_command)
            r.sleep()
        

if __name__ == '__main__':
    s = Steering()
    s.run()
