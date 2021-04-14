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

from std_msgs.msg import Byte, Bool, Int16, String, Int8MultiArray
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Point
import geometry_msgs.msg


import time

######################################################################
######################################################################
# By using this node!
## as a trigger.
### rostopic pub wstation/lift_item_size std_msgs/String "data: 'good'"
## as a connection.
### rostopic echo /room_num
### rostopic echo /floor_num
######################################################################
######################################################################

### 본 script 는 Azure Kinect 를 이용하여, 운송장을 촬영하고,
### 촬영한 운송장을 네이버 OCR API 를 호출하여, 기능을 수행한 후,
### 아파트의 동, 호수 정보를 parsing 하는 script 이다.

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
	
def main():

	while(True):

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
				# print(contourSize)
				# print(paperSize)
				# print(ratio)

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
			# cv2.waitKey(50)

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
			cv2.destroyAllWindows()

			break
	
	cv2.imwrite('/home/ws/catkin_ws/src/scan.png',result_im)
	# cv2.waitKey(0)
	cv2.destroyAllWindows()
	global img 
	# 카메라로 촬영한 사진 열기.
	with open("/home/ws/catkin_ws/src/scan.png", "rb") as f:
	    img = base64.b64encode(f.read())

	cv2.destroyAllWindows()
	naver_ocr()

def naver_ocr() :

	# 토픽으로 층, 호수 정보 publish
	# pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
	floor_pub = rospy.Publisher("/floor_num", Int16, queue_size=1)
	room_pub = rospy.Publisher("/room_num", Int16, queue_size=1)

	floor_num = Int16()
	room_num = Int16()

	# rospy.init_node('floor_and_room_publisher', anonymous=True)
	rate = rospy.Rate(10)


	# 네이버 OCR 을 위한 URL 과 KEY 그리고 json 양식
	URL = "https://294fcbd76a9b4e958d42600f6fef8b80.apigw.ntruss.com/custom/v1/6633/3082721950f37db2302acfa2abfdb02b9931bc77ab2e1bc2d3d4a0b6ca676961/general"
	
	KEY = "R2lzanBXZFBRdEZjbkhhYmdVZlBTc2ZwSlh5Tnhieno="

	headers = {
	    "Content-Type": "application/json",
	    "X-OCR-SECRET": KEY
	}
	data = {
	    "version": "V1",
	    "requestId": "sample_id", 
	    "timestamp": 0,
	    "images": [
	        {
	            "name": "scan",
	            "format": "png",
	            "data": img.decode('utf-8')
	        }
	    ]
	}

	data = json.dumps(data)
	# URL, KEY 를 통해 네이버 OCR을 호출한다. output 은 json 형식으로 추출된다. 
	response = requests.post(URL, data=data, headers=headers)
	res = json.loads(response.text)

	# 추출된 json data 를 파일로 만들어준다.(주의)ensure_ascii = false 라고 해주지 않으면, 한글이 깨진다.
	with io.open('/home/ws/catkin_ws/src/1.json', 'w') as make_file:
	    json.dump(response.text, make_file, ensure_ascii=False, indent=2)

	# json 파일을 열고, OCR 된 text-data 들을 list 로 추출한다.
	with io.open('/home/ws/catkin_ws/src/1.json','r') as f:
	    json_data = json.load(f)

	# 한글 데이터를 불러오기 위해 encoding= utf-8 을 해주었다.
	res = json.loads(json_data, encoding="utf-8")

	# 추출되는 OCR Text data 를 list 에 넣는다.
	list_arr = list()
	length = len(res['images'][0]['fields'])
	for i in range(length):
	    list_arr.append(res['images'][0]['fields'][i]['inferText'].encode('utf-8'))


	# 추출된 데이터 중에서도 아파트의 동, 호수를 parsing 한다.
	# 이때, 단어의 시작이 숫자인지의 여부를 통해 다른 유사어(고잔동, 사동..)가 아닌
	# 아파트의 동, 호수를 추출할 수 있게 제한을 두었다.
	# 또한, OCR 진행 시, 동과 호수가 같이 기술되어 있는 경우가 있다.
	# 이때는, 대부분 순서가 동, 호수 순이기 때문에, 문자들중 숫자만 추출하고,순서대로 배열에 넣는다.
	# 뒤에 숫자를 호수라고 판단하고, 

	resultlist = []
	resultlist_ = []

	_room_point = []
	room_split = ['0','0','0','0']

	for row in list_arr:

		if "호" in row:
			if row[0].isdigit() :
				room_ = filter(str.isdigit,row)
				resultlist.append(room_)

				# info.csv 에 있는 우리 서비스를 사용하는 사람의 택배만을 핸들링하기 위한 절차.
				with open('/home/ws/catkin_ws/src/Azure_Kinect_ROS_Driver/src/info.csv', 'r') as file:
					reader = csv.reader(file, delimiter = ',')
					num = 0
					for row in reader:
						_room_point.append(int(row[1]))
						num = num + 1
					print(_room_point)
					print(room_)
					search_result = -1

					for i in range(num) :
						if str(_room_point[i]).find(room_) != -1 :
							search_result = 0

					if search_result == -1 :
						print("This parcel is not on our service!!")
					else :
						print("find")


				# global first_try
				# global second_try
				# # global trial

				# floor = 0
				# room = 0


				# 층, 호수 구분
				# 10 층 이상과 이하의 경우를 따로 나누어 계산한다.
				if len(room_) == 3 : 
					for i in range(3) :
						room_split[i+1] = str(room_[i])
				elif len(room_) == 4 :
					for i in range(4) :
						room_split[i] = str(room_[i])

				global floor
				global room
				floor = int(room_split[0] + room_split[1])
				room = int(room_split[2] + room_split[3])

				# Saving floor and room info in csv file
				resultlist_.append(floor)
				resultlist_.append(room)

				# topic publish
				floor_num.data = int(floor)
				room_num.data = int(room)
				
				while not rospy.is_shutdown():
					
					floor_connections = floor_pub.get_num_connections()
					room_connections = room_pub.get_num_connections()

					if floor_connections > 0 :
						floor_pub.publish(floor_num)
						if room_connections > 0 :
							room_pub.publish(room_num)
							break

        			rate.sleep()

	    # '-' 의 경우 송장에서 쓰이는 경우가 너무 많아 특별한 제약을 걸지 않는 한 쓸 수 없을 듯 하다.
	    # elif "-" in row:
	    #     if row[0].isdigit() :
	    #         resultlist.append(row)
		# 추출한 숫자가 row 의 몇번째에 있는지 알고 싶을 때...
	    #         apt_num = list_arr.index(row)

	##########################################################
	##########################################################
	# 사용환경에 따라, 저장 하는 주소 바꿔주기!!!!!!
	##########################################################
	##########################################################

	# 아파트의 동, 호수 정보를 숫자로만 저장한다.
	with open('/home/ws/catkin_ws/src/2.csv', 'w') as f:
	    writer = csv.writer(f)
	    writer.writerow(resultlist)

	# 집의 층과 호수 정보만을 따로 추출해서 저장한다.
	with open('/home/ws/catkin_ws/src/3.csv', 'w') as f:
	    writer = csv.writer(f)
	    writer.writerow(resultlist_)

def item_status_cb(data):
	global item_status_data
	item_status_data = data.data

def video_callback(data) :
	bridge = CvBridge()
	global frame
	frame = bridge.imgmsg_to_cv2(data)
	# cv2.imshow("camera", frame)
	# rospy.loginfo("receiving video frame")

def init_callback(data) :
	if data.data == True :
		ocr_status.data = False	

if __name__ == '__main__':

	rospy.init_node("ocr_status", anonymous=True)

	video_subscriber = rospy.Subscriber("/rgb/image_raw", Image, video_callback)
	init_subscriber = rospy.Subscriber("/initialize", Bool, init_callback)
	ocr_publisher = rospy.Publisher("/ocr_status", Bool, queue_size=1)
	ocr_status = Bool()

	while not rospy.is_shutdown():
		item_status_data = "none"
		item_status = rospy.Subscriber("/wstation/lift_item_size", String, item_status_cb)

		if item_status_data == "good" :
			start = main()
			print 
			print "over!! :)"
			print

			if (int(floor) < 1) | (int(room) < 1) :
				print("again")
				ocr_status.data = False
				again = main()
			else :
				ocr_status.data = True
				print("ocr succeed at once!")
				print
			
		ocr_publisher.publish(ocr_status)
				
		# else :
			# print
			# print("not yet")
			# print
