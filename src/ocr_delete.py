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
	global img
	
	img_origin = cv2.imread("/home/minwoo/catkin_ws/src/scan.png",cv2.IMREAD_COLOR)
	
	_height, _width, channel = img_origin.shape
	
	print("again")
	print(trial)
	if (trial == 2) :
		img180 = cv2.rotate(img_origin, cv2.ROTATE_180)
		cv2.imwrite('/home/minwoo/catkin_ws/src/scan.png',img180)
	# else :
	# 	if _height > _width  :
	# 		img90 = cv2.rotate(img_origin, cv2.ROTATE_90_CLOCKWISE)
	# 		cv2.imwrite('/home/minwoo/catkin_ws/src/scan.png',img90)
	# 		print("wrong rotation")
	# 	else : 
	# 		print("correct rotation")


	# 카메라로 촬영한 사진 열기.
	with open("/home/minwoo/catkin_ws/src/scan.png", "rb") as f:
	    img = base64.b64encode(f.read())

	naver_ocr()

def naver_ocr() :

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
	with io.open('/home/minwoo/catkin_ws/src/1.json', 'w') as make_file:
	    json.dump(response.text, make_file, ensure_ascii=False, indent=2)

	# json 파일을 열고, OCR 된 text-data 들을 list 로 추출한다.
	with io.open('/home/minwoo/catkin_ws/src/1.json','r') as f:
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

	global floor
	global room

	floor = 0
	room = 0

	for row in list_arr:

		if "호" in row:
			if row[0].isdigit() :
				room_ = filter(str.isdigit,row)
				resultlist.append(room_)

				# info.csv 에 있는 우리 서비스를 사용하는 사람의 택배만을 핸들링하기 위한 절차.
				with open('/home/minwoo/catkin_ws/src/Azure_Kinect_ROS_Driver/src/info.csv', 'r') as file:
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

				# 층, 호수 구분
				# 10 층 이상과 이하의 경우를 따로 나누어 계산한다.
				if len(room_) == 3 : 
					for i in range(3) :
						room_split[i+1] = str(room_[i])
				elif len(room_) == 4 :
					for i in range(4) :
						room_split[i] = str(room_[i])

				floor = int(room_split[0] + room_split[1])
				room = int(room_split[2] + room_split[3])

				# Saving floor and room info in csv file
				resultlist_.append(floor)
				resultlist_.append(room)
				
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
	with open('/home/minwoo/catkin_ws/src/2.csv', 'w') as f:
	    writer = csv.writer(f)
	    writer.writerow(resultlist)

	# 집의 층과 호수 정보만을 따로 추출해서 저장한다.
	with open('/home/minwoo/catkin_ws/src/3.csv', 'w') as f:
	    writer = csv.writer(f)
	    writer.writerow(resultlist_)

def item_status_cb(data):
	global item_status_data
	item_status_data = data.data
	
	
def init_callback(data) :
	if data.data == True :
		floor_num.data = 0
		room_num.data = 0
		ocr_status.data = "none"

def trigger_cb(data) :
	global trigger
	trigger = data.data


if __name__ == '__main__':

	rospy.init_node("ocr_status", anonymous=True)

	ocr_publisher = rospy.Publisher("/wstation/ocr_status", String, queue_size=1)
	ocr_status = String()

	floor_pub = rospy.Publisher("/floor_num", Int16, queue_size=1)
	room_pub = rospy.Publisher("/room_num", Int16, queue_size=1)

	floor_num = Int16()
	room_num = Int16()

	global trial

	while not rospy.is_shutdown():
		item_status_data = "none"

		trigger = rospy.Subscriber("/wstation/start_ocr_estimation", Bool, trigger_cb, queue_size=1)
		
		item_status = rospy.Subscriber("/wstation/lift_item_size", String, item_status_cb, queue_size=1)

		init_subscriber = rospy.Subscriber("/initialize", Bool, init_callback)
		
		if trigger == True :
			if item_status_data == "good" :
				
				ocr_status.data = "trigger_off"
				ocr_publisher.publish(ocr_status)
				rospy.sleep(2.5)
				
				trial = 1
				
				start = main()
				print 
				print "over!! :)"
				print

				if (int(floor) > 1) & (int(room) > 1) :
					print("ocr succeed at once!")
					ocr_status.data = "good"
					floor_num.data = int(floor)
					room_num.data = int(room)

				else :
					trial = 2
					print("again")
					again = main()
					if (int(floor) > 1) & (int(room) > 1) :
						ocr_status.data = "good"
						print("ocr succeed!")
					else :
						ocr_status.data = "bad"
						print("ocr failed")

			
		ocr_publisher.publish(ocr_status)
		floor_pub.publish(floor_num)
		room_pub.publish(room_num)

