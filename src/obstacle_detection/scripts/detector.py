#!/usr/bin/env python
import sys
import numpy as np
from scipy.ndimage import filters
import cv2
import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pickle
import os.path

class detector:
	def __init__(self):
		self.subscriber = rospy.Subscriber("/camera/depth/image", Image, self.callback)
		self.bridge = CvBridge()

	def callback(self, data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, data.encoding)
		except CvPrindgeError as e:
			print(e)
		print(data.encoding)
		if not os.path.isfile("depth_image.pkl"):
			with open("depth_image.pkl", 'wb') as f:
				pickle.dump(image,f)

def main(args):
	detect = detector()
	rospy.init_node('detector', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Bye")

if __name__ == '__main__':
	main(sys.argv)
