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



def main():

	# 토픽으로 이미지 확인 할 수 있게 확인 차... 던져야 하는 토픽은 추후에 보완할 예정..
	# 예를 들면, 층, 호수 정보를 csv 파일이 아닌, topic 으로 던져라 라던가..
	# pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
	floor_pub = rospy.Publisher("/floor_num", Int16, queue_size=1)
	room_pub = rospy.Publisher("/room_num", Int16, queue_size=1)

	floor_num = Int16()
	room_num = Int16()

	rospy.init_node('floor_and_room_publisher', anonymous=True)
	rate = rospy.Rate(10)

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

	_room_point = []
	room_split = ['0','0','0','0']

	floor = None 
	room = None

	for row in list_arr:

		if "호" in row:
			if row[0].isdigit() :
				room_ = filter(str.isdigit,row)
				resultlist.append(room_)

				# info.csv 에 있는 우리 서비스를 사용하는 사람의 택배만을 핸들링하기 위한 절차.
				with open('/home/minhye/catkin_ws/src/Azure_Kinect_ROS_Driver/src/info.csv', 'r') as file:
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
							print("find")
							search_result = 0

					if search_result == -1 :
						print("This parcel is not on our service!!")


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
	    #         apt_num = list_arr.index(row)

	##########################################################
	##########################################################
	# 사용환경에 따라, 저장 하는 주소 바꿔주기!!!!!!
	##########################################################
	##########################################################

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
    except Exception: 
		pass