#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np



class TestWarper:
	def __init__(self):
		rospy.init_node('test_warper', anonymous=True)
		self.sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.callback, queue_size=1)
		
		h = 480
		w = 640
		print("h : " ,h)
		print("w : " ,w)

		# distort scr to dst
		# src = np.float32([
		#     [w * 1.6, h * 1.3],
		#     [w * (-0.1), h * 1.3],
		#     [0, h * 0.62],
		#     [w, h * 0.62],
		# ])
		self.src = np.float32([
			[480*1.6, 530*1.3],
			[480*(-0.1), 530*1.3],
			[0, h*0.62],
			[w, h*0.62],
		])
		self.dst = np.float32([
			[w * 0.65, h],
			[w * 0.35, h],
			[-300, 0],
			[w+300, 0],
		])
		# dst = np.float32([
		#     [0, 0],
		#     [w, 0],
		#     [0, h],
		#     [w , h],
		# ])
		self.M = cv2.getPerspectiveTransform(self.src, self.dst)
		self.Minv = cv2.getPerspectiveTransform(self.dst, self.src)
		rospy.spin()

	def warp(self, img):
		return cv2.warpPerspective(
			img,
			self.M,
			(img.shape[1], img.shape[0]),
			flags=cv2.INTER_LINEAR
		)

	def unwarp(self, img):
		return cv2.warpPersective(
			img,
			self.Minv,
			(img.shape[1], img.shape[0]),
			flags=cv2.INTER_LINEAR
		)
	

	def callback(self, data):
		img_bgr = cv2.imdecode(np.fromstring(data.data, np.uint8), cv2.IMREAD_COLOR)
		warp = self.warp(img_bgr)
		cv2.imshow("Test Warper", warp)
		cv2.waitKey(1)

if __name__ == '__main__':
	try:
		test_warper = TestWarper()

	except rospy.ROSInterruptException:
		pass
