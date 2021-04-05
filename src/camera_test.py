#!/usr/bin/env python
#_*_ coding: utf-8 _*_

import json
import base64
import requests
import sys
import csv
import io
import re
import rospy
import cv2
import numpy as np

from std_msgs.msg import Byte, Bool, Int16, String
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Point
import geometry_msgs.msg


import time

### 본 script 는 Azure Kinect 를 이용하여, 운송장을 촬영하고,
### 촬영한 운송장을 네이버 OCR API 를 호출하여, 기능을 수행한 후,
### 아파트의 동, 호수 정보를 parsing 하는 script 이다.


# Azure Kinect 경우 웹캠이 있는 노트북으로 사용할 때에는, 4번으로 설정해주어야한다. 컴퓨터의 경우 0 번을 해주면 된다.
# bridge 는 Opencv 와 ROS 의 연결을 위해 필요한것이다.

bridge = CvBridge()
video_capture1 = cv2.VideoCapture(0)


# Opencv 로 받아온 윤곽의 point 를 rect 배열로 변환시켜준다.
def order_points(pts):
	rect = np.zeros((4, 2), dtype = "float32")
	s = pts.sum(axis = 1)
	rect[0] = pts[np.argmin(s)]
	rect[2] = pts[np.argmax(s)]
	diff = np.diff(pts, axis = 1)
	rect[1] = pts[np.argmin(diff)]
	rect[3] = pts[np.argmax(diff)]

	return rect

# 카메라가 켜졌는지 여부를 알려주는 함수.
def OpenVideo():
	Test = False
	if video_capture1.isOpened():
		print 'it is opened successfully!'
		Test = True
	else:
		print 'cannot open it!'
	return Test



def main():

	# 카메라 ON
	Test = OpenVideo()

	# 토픽으로 이미지 확인 할 수 있게 확인 차... 던져야 하는 토픽은 추후에 보완할 예정..
	# 예를 들면, 층, 호수 정보를 csv 파일이 아닌, topic 으로 던져라 라던가..
	# pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
	floor_pub = rospy.Publisher("/floor_num", Int16, queue_size=1)
	room_pub = rospy.Publisher("/room_num", Int16, queue_size=1)

	floor_num = Int16()
	room_num = Int16()

	rospy.init_node('floor_and_room_publisher', anonymous=True)
	rate = rospy.Rate(10)


	while(True):

		ret, frame = video_capture1.read()

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

		# 윤각 잡기 편하도록 gray 이미지로 바꾼 후 진행. GaussianBlur 를 통해 
		# 컴퓨터가 이미지 처리를 편하게 하게끔 해준다.
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
			result_im = cv2.resize(varped, (1200,1000))


			print("apply perspective transform.")

			# varped = cv2.cvtColor(varped, cv2.COLOR_BGR2GRAY)
			# varped = cv2.adaptiveThreshold(varped, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, 10)

			break
	
	video_capture1.release()
	cv2.destroyAllWindows()

	cv2.imshow("scan data" ,result_im)
	cv2.imwrite('/home/minhye/catkin_ws/src/scan.png',result_im)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

	# 카메라로 촬영한 사진 열기.
	with open("/home/minhye/catkin_ws/src/scan.png", "rb") as f:
	    img = base64.b64encode(f.read())



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



