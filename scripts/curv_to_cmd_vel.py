#!/usr/bin/python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class Control:
    def __init__(self, pub_topic='/cmd_vel'):
        self.cmd_vel = rospy.Publisher(pub_topic, Twist, queue_size=2)
        rospy.init_node('cmd_vel_converter', anonymous=True)
        rospy.loginfo("control pub topic\t{}".format(pub_topic))
        self.curv = 0
        self.vel = 0
        self.ready = False
        rospy.Subscriber('/velocity_command', Float32, self.vel_cb)
        rospy.Subscriber('/curvature_command', Float32, self.curv_cb)

    def vel_cb(self, msg):
        self.vel = msg.data
        if self.ready:
            self.calc_ang_vel()
        else:
            self.ready = True


    def curv_cb(self,msg):
        self.curv = msg.data
        if self.ready:
            self.calc_ang_vel()
        else:
            self.ready = True

    def calc_ang_vel(self):
        ang_vel = self.curv * self.vel
        msg = Twist()
        msg.linear.x = self.vel
        msg.angular.z = ang_vel
        self.cmd_vel.publish(msg)



if __name__ == '__main__':
    c = Control(pub_topic='/cmd_vel')
    rospy.spin()



