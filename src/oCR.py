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


# Azure Kinect 경우 웹캠이 있는 노트북으로 사용할 때에는, 4번으로 설정해주어야한다.
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
	with io.open('/home/minhye/catkin_ws/src/1.json', 'w') as make_file:
	    json.dump(response.text, make_file, ensure_ascii=False, indent=2)

	# json 파일을 열고, OCR 된 text-data 들을 list 로 추출한다.
	with io.open('/home/minhye/catkin_ws/src/1.json','r') as f:
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

	floor = None 
	room = None

	for row in list_arr:

		if "동" in row and "호" in row:
			apt_room = [int(d) for d in re.findall(r'-?\d+', row)]
			resultlist.append(apt_room)

			# 층, 호수 구분
			b_room_split = str(apt_room[1]).split('0')
			floor = b_room_split[0]
			room = b_room_split[1]
			resultlist_.append(floor)
			resultlist_.append(room)
			room_num = list_arr.index(row)

		elif "동" in row:
			if row[0].isdigit() :
				apt = filter(str.isdigit,row)
				resultlist.append(apt)
				# resultlist.append(row)
				apt_num = list_arr.index(row)
				
		elif "호" in row:
			if row[0].isdigit() :
				room_ = filter(str.isdigit,row)
				resultlist.append(room_)

				# 층, 호수 구분
				room_split = room_.split('0')
				floor = room_split[0]
				room = room_split[1]
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
					

 
				room_num = list_arr.index(row)


	    # '-' 의 경우 송장에서 쓰이는 경우가 너무 많아 특별한 제약을 걸지 않는 한 쓸 수 없을 듯 하다.
	    # elif "-" in row:
	    #     if row[0].isdigit() :
	    #         resultlist.append(row)
	    #         apt_num = list_arr.index(row)


	# 아파트의 동, 호수 정보를 숫자로만 저장한다.
	with open('/home/minhye/catkin_ws/src/2.csv', 'w') as f:
	    writer = csv.writer(f)
	    writer.writerow(resultlist)

	# 집의 층과 호수 정보만을 따로 추출해서 저장한다.
	with open('/home/minhye/catkin_ws/src/3.csv', 'w') as f:
	    writer = csv.writer(f)
	    writer.writerow(resultlist_)




if __name__ == '__main__':
    try:
        start = main()
        # rospy.spin() 
        print 
        print "over!! :)"
        print
    except Exception: 
		pass