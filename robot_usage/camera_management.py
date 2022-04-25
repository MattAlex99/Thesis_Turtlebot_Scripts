from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class TakePhoto:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/raspicam_node/image/compressed"
        self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.callback)
        # Allow up to one second to connection
        rospy.sleep(1)

    def take_picture(self):
        #return True, cv2.imread("/home/alexander/Thesis_Turtlebot_Scripts/test_images/angleV9_6.jpg")
        if self.image_received:
            return True,self.image
        else:
            return False,None

    def callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.image_received = True
        self.image = cv_image