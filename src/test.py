#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import sys

from std_msgs.msg import Byte, Bool, Int16, String
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Point
import geometry_msgs.msg


import time


bridge = CvBridge()
# video_capture1 = cv2.VideoCapture('/home/minhye/catkin_ws/src/Azure_Kinect_ROS_Driver/src/image/test_video_1.mp4')
video_capture1 = cv2.VideoCapture(4)
# video_capture1 = cv2.VideoCapture(1)


def order_points(pts):
	rect = np.zeros((4, 2), dtype = "float32")
	s = pts.sum(axis = 1)
	rect[0] = pts[np.argmin(s)]
	rect[2] = pts[np.argmax(s)]
	diff = np.diff(pts, axis = 1)
	rect[1] = pts[np.argmin(diff)]
	rect[3] = pts[np.argmax(diff)]

	return rect


def OpenVideo():
	#video_capture1 = cv2.VideoCapture(0)
	Test = False
	if video_capture1.isOpened():
		print 'it is opened successfully!'
		Test = True
	else:
		print 'cannot open it!'
	return Test

# def order_points(pts):



def main():
	Test = OpenVideo()

	pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

	rospy.init_node('image_publisher', anonymous=True)
	# rate = rospy.Rate(1)


	while(True):
		ret, frame = video_capture1.read()
		# cv2.imshow("Frame",frame)

		# add your file here
		# frame_ros = bridge.cv2_to_imgmsg(frame, "bgr8")
		# pub.publish(frame_ros)


		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (3,3), 0)
		edged = cv2.Canny(gray, 75, 200)

		print("Edge Detection start")


		(_,cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		cnts = sorted(cnts, key = cv2.contourArea, reverse=True)

		for c in cnts:
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.02* peri, True)
			# print("approx :")
			# print(approx)
			screenCnt = []

			if len(approx) == 4:
				contourSize = cv2.contourArea(approx)
				paperSize = 200 # 21.0 * 29.7
				ratio = contourSize / paperSize
				print(contourSize)
				print(paperSize)
				print(ratio)

				if ratio > 200 : 
					screenCnt = approx
				break

		if len(screenCnt) == 0:
			print("fail to get edge")
			cv2.imshow("Frame", frame)
			continue
		else:
			print("show contour of paper")
			cv2.waitKey(1000)

			cv2.drawContours(frame, [screenCnt], -1, (0, 255, 0), 2)
			cv2.imshow("Frame", frame)

			cv2.waitKey(50)


			rect = order_points(screenCnt.reshape(4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = rect
			v1 = abs(bottomRight[0] - bottomLeft[0])
			v2 = abs(topRight[0] - topLeft[0])
			h1 = abs(topRight[1] - bottomRight[1])
			h2 = abs(topLeft[1] - bottomLeft[1])
			maxWidth = max([v1, v2])
			maxHeight = max([h1, h2])

			dst = np.float32([[0,0], [maxWidth-1, 0], [maxWidth-1, maxHeight-1], [0, maxHeight-1]])

			N = cv2.getPerspectiveTransform(rect, dst)
			varped = cv2.warpPerspective(frame, N, (maxWidth, maxHeight))

			print("apply perspective transform.")

			# varped = cv2.cvtColor(varped, cv2.COLOR_BGR2GRAY)
			# varped = cv2.adaptiveThreshold(varped, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, 10)

			break
	
	video_capture1.release()
	cv2.destroyAllWindows()

	cv2.imshow("scan data" ,varped)
	cv2.imwrite('/home/minhye/catkin_ws/src/scan.png', varped)
	print("watt!~")
	cv2.waitKey(0)
	cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        start = main()

        # rospy.spin() 
        print 
        print "over!! :)"
        print
    except Exception: 
		pass



# def callback(data):
#     _irdata = Int16()
#     _irdata = data
#     irdata = str(binary(_irdata.data))
#     rate = rospy.Rate(40)

#     ostr_top = "[IR_STATE ] " 

#     ostr_top += " L" if irdata[0:1] == '1' else " ~" 
#     ostr_top += "|C" if irdata[1:2] == '1' else "|~" 
#     ostr_top += "|R" if irdata[2:3] == '1' else "|~" 
#     ostr_top += " "
#     ostr_top += " L" if irdata[3:4] == '1' else " ~" 
#     ostr_top += "|C" if irdata[4:5] == '1' else "|~" 
#     ostr_top += "|R" if irdata[5:6] == '1' else "|~" 
#     ostr_top += " "
#     ostr_top += " L" if irdata[6:7] == '1' else " ~" 
#     ostr_top += "|C" if irdata[7:8] == '1' else "|~" 
#     ostr_top += "|R" if irdata[8:9] == '1' else "|~" 
#     ostr_top += " "

#     print ostr_top
#     print(irdata)
#     print
#     rate.sleep()


# def __init__():
  
    # rospy.init_node("dock_ir_intepreter", anonymous=True)
    # _irSubscriber = rospy.Subscriber('/dockdata_T', Int16, callback)



# def binary(n):
#     bin_data = ''
#     for i in range(0,9):
#         if n&1 : 
#             bin_data = '1' + bin_data
#         else:
#             bin_data = '0' + bin_data
#         n = n >> 1
#     return bin_data
    


# if __name__ == '__main__':
#     try:
#         # __init__()

#         # rospy.spin() 
#         # print 
#         # print "It converts dock_ir data to human friendly format."
#         # print
#     except rospy.ROSInterruptException: pass



