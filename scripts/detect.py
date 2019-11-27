#!/usr/bin/python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from line_follower import line_detector


class Detector:
    def __init__(self):
        self.img_pub = rospy.Publisher('detected_image', Image, queue_size=2)
        self.line_pub = rospy.Publisher('line_position', Float32, queue_size=2)
        rospy.init_node('detector', anonymous=True)
        rospy.Subscriber('/camera/image_raw', Image, self.image_cb)
        self.bridge = CvBridge()

    def image_cb(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_img, cx = self.process_image(cv_img)
        ros_img = self.bridge.cv2_to_imgmsg(cv_img, "mono8")
        self.img_pub.publish(ros_img)
        self.line_pub.publish(cx)

    def process_image(self, cv_img):
        xrange = (0.0, 1.0)
        yrange = (0.8, 1.0)
        cv_img = line_detector.crop(cv_img, xrange, yrange)
        cv_img = line_detector.to_gray(cv_img)
        cv_img = line_detector.do_threshold(cv_img)
        cv_img, contours = line_detector.get_contours(cv_img)
        cv_img, cx = line_detector.draw_contours(cv_img, contours)
        return (cv_img, cx)


if __name__ == '__main__':
    d = Detector()
    rospy.spin()
