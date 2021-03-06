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
from matplotlib import pyplot as plt

from std_msgs.msg import Byte, Bool, Int16, String, Int8MultiArray
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Point
import geometry_msgs.msg


import time

SAVE_PATH = "/home/seunghwan/catkin_ws/src/"

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
		ret,thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)		# added by s
		edged = cv2.Canny(thresh, 75, 200)

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
				paperSize = 200 #200 # 21.0 * 29.7
				ratio = contourSize / paperSize
				# print(contourSize)
				# print(paperSize)
				# print(ratio)

				if ratio > 200 : # 200
					screenCnt = approx
				break

		if len(screenCnt) == 0:
			print("fail to get edge")
			cv2.imshow("Frame", frame)
			cv2.imshow("edged", edged)
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
			check = 1
			if maxHeight > maxWidth :
				check = 2
				maxHeight = maxWidth
				maxWidth = maxHeight
				img90 = cv2.rotate(varped, cv2.ROTATE_90_CLOCKWISE)

			m_w = 1300 / maxWidth 
			m_h = 1000 / maxHeight 

			max_mul = max(m_h,m_w)


			if check == 2 :
				result_im = cv2.resize(img90, (int(maxWidth*max_mul),int(maxHeight*max_mul)))
			elif check == 1 : 	
				result_im = cv2.resize(varped, (int(maxWidth*max_mul),int(maxHeight*max_mul)))

			print("apply perspective transform.")
			# varped = cv2.cvtColor(varped, cv2.COLOR_BGR2GRAY)
			# varped = cv2.adaptiveThreshold(varped, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, 10)
			cv2.destroyAllWindows()

			break
	
	if trial == 2 :
		img180 = cv2.rotate(result_im, cv2.ROTATE_180)
		cv2.imwrite(SAVE_PATH + 'scan.png',img180)
		# cv2.imwrite('/home/ws/catkin_ws/src/scan.png',img180)
	else : 
		cv2.imwrite(SAVE_PATH + 'scan.png',result_im)
		# cv2.imwrite('/home/ws/catkin_ws/src/scan.png',result_im)

	# cv2.waitKey(0)
	cv2.destroyAllWindows()
	global img 
	# 카메라로 촬영한 사진 열기.
	with open(SAVE_PATH + "scan.png", "rb") as f:
	    img = base64.b64encode(f.read())

	cv2.destroyAllWindows()
	naver_ocr()

def scanner():    
	global frame 

	while (True):
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

		clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
		original = frame.copy()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		contrast = clahe.apply(gray)

		# blurred = cv2.medianBlur(thresh, 21)
		resContrast = cv2.GaussianBlur(contrast, (3,3), 0)
		ret,thresh = cv2.threshold(resContrast, 200, 255, cv2.THRESH_BINARY)

		canny = cv2.Canny(thresh, 75, 200)

		dialated = cv2.dilate(canny, cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)), iterations = 3)
		closing = cv2.morphologyEx(dialated, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8),iterations = 10)
		contimage, contours, hierarchy = cv2.findContours(closing, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		contours = sorted(contours, key = cv2.contourArea, reverse = True)[:5]
		target = None
		
		for c in contours:
			p = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.09 * p, True)
			
			if len(approx) == 4:
				target = approx
				cv2.drawContours(frame, [target], -1, (0, 255, 0), 2)
				break
		
		cv2.imshow('frame', frame)
		# plt.figure(figsize = (20,20))
		# plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
		# plt.title("final")
		# plt.show()

	cv2.destroyAllWindows()

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
	with io.open(SAVE_PATH + '1.json', 'w') as make_file:
	    json.dump(response.text, make_file, ensure_ascii=False, indent=2)

	# json 파일을 열고, OCR 된 text-data 들을 list 로 추출한다.
	with io.open(SAVE_PATH + '1.json','r') as f:
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
				with open(SAVE_PATH + 'Azure_Kinect_ROS_Driver/src/info.csv', 'r') as file:
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
	with open(SAVE_PATH + '2.csv', 'w') as f:
	    writer = csv.writer(f)
	    writer.writerow(resultlist)

	# 집의 층과 호수 정보만을 따로 추출해서 저장한다.
	with open(SAVE_PATH + '3.csv', 'w') as f:
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
		floor_num.data = 0
		room_num.data = 0
		ocr_status.data = "none"

def trigger_cb(data) :
	global trigger
	trigger = data.data


if __name__ == '__main__':

	rospy.init_node("ocr_status", anonymous=True)

	video_subscriber = rospy.Subscriber("/rgb/image_raw", Image, video_callback)

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
				print("hihi")
			# if item_status_data == "good" :
				
				ocr_status.data = "trigger_off"
				ocr_publisher.publish(ocr_status)
				rospy.sleep(2.5)
				
				trial = 1

				# start = main()
				scanner()
				
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
					
					# again = main()
					scanner()

					if (int(floor) > 1) & (int(room) > 1) :
						ocr_status.data = "good"
						print("ocr succeed!")
					else :
						ocr_status.data = "bad"
						print("ocr failed")

			
		ocr_publisher.publish(ocr_status)
		floor_pub.publish(floor_num)
		room_pub.publish(room_num)

