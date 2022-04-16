#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import numpy as np

class Linefollower:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.image_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.twist = Twist()

    def image_cb(self, msg):
        # convert compressedimage to hsv
        np_arr = np.fromstring(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # mask off all color except yellow
        lower_yellow = numpy.array([40, 0, 0])
        upper_yellow = numpy.array([120, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(image, image, mask=mask)

        # retain a 30 pixel band near the bottom of the image
        h, w, d = image.shape
        search_top = int(3 * h /4)
        search_bot = search_top + 30
        mask[0:search_top, :] = 0
        mask[search_bot:h, :] = 0

        # turn band into cv2 moments
        M = cv2.moments(mask)
        moment_sum = sum(M.values())

        # if the line is present in the mask, find the centroid and turn towards it
        if moment_sum > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            err = cx - w/2
            self.twist.linear.x = 0.1
            self.twist.angular.z = -float(err) / 1000
            self.cmd_vel_pub.publish(self.twist)
        # if no line is present in the mask, turn the robot around
        else:
            self.twist.angular.z = 0.2
            self.twist.linear.x = 0
            self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("band", mask)
        cv2.imshow("masked", masked)
        cv2.imshow("original", image)
        cv2.waitKey(3)

#init node
rospy.init_node("linefollow")
follower = Linefollower()
rospy.spin()