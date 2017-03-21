#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers

bridge = CvBridge()

known_width = 6.5 #Diameter in cm of the circunference
wpix = 116 #Inicial diameter in cm of the circunference
d = 34 #Inicial dist from the camera to the circunference in cm
focal_dist = (wpix * d) / known_width

cv_image = None
media = []
centro = [320, 240]
atraso = 1.5
vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
z = 100
z_desejado = 80

def processa(dado):
	global media
	global centro
	frame = dado
	frame_r = frame[:,:,2]
	frame_g = frame[:,:,1]
	frame_rg = cv2.subtract(frame_r, frame_g)

	ret, frame_rg = cv2.threshold(frame_rg, 125, 255, cv2.THRESH_BINARY)
	bordas = cv2.morphologyEx(frame_rg, cv2.MORPH_GRADIENT, np.ones((3, 3)) )
	contornos, arvore = cv2.findContours(bordas, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	
	maior_contorno = None
	maior_contorno_area = 0
	# imagem de saida
	disp = cv2.cvtColor(frame_rg, cv2.COLOR_GRAY2BGR)
    	for cnt in contornos:
    	    area = cv2.contourArea(cnt)
	    
z = (known_width * focal_dist) / (2 * cnt[2])
    	    if area > maior_contorno_area:
    	        maior_contorno = cnt
    	        maior_contorno_area = area
    
    	# Encontramos o centro do contorno fazendo a média de todos seus pontos.
    	if not maior_contorno is None :
    	    cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
    	    maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
    	    media = maior_contorno.mean(axis=0)
    	    media = media.astype(np.int32)
    	    cv2.circle(frame, tuple(media), 5, [0, 255, 0])
    	else:
            media = []
			
	cv2.imshow('caixa', disp)

	cv2.waitKey(1)

def recebe(imagem):
	global cv_image
	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime

	delay = (lag.secs+lag.nsecs/1000000000.0)
    
	if delay > atraso:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.imgmsg_to_cv2(imagem, "bgr8")
		cv2.imshow("video", cv_image)
		cv2.waitKey(1)
		processa(cv_image)
		depois = time.clock()
	except CvBridgeError as e:
		print(e)

def Movimento(dif_x):
	global media
	print(dif_x)
	global z
	
	if z > z_desejado: 	
		if math.fabs(dif_x)<30 :
			print('seguindo') 
			return Twist(Vector3(0.2,0,0), Vector3(0,0,0))

		else:	
			if dif_x > 0:
				# Vira a direita
				print('direita')
				return Twist(Vector3(0.2,0,0), Vector3(0,0,-0.3))
			else:
				# Vira a esquerda
				print('esquerda')
				return Twist(Vector3(0.2,0,0), Vector3(0,0,0.3))
	else:	
		print('Alvejadooooooooooooooooooooooooooooooooooooooooooooo')		
		return  Twist(Vector3(0.0,0,0), Vector3(0,0,0))
			

def Procura():
	print('Procurando')
	return Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
	

if __name__=="__main__":

	rospy.init_node("cor")
	recebedor = rospy.Subscriber("/camera/image_raw", Image, recebe, queue_size=10, buff_size = 2**24)
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	cv2.namedWindow("caixa")
	cv2.namedWindow("video")

	try:

		while not rospy.is_shutdown():

			if len(media) != 0 and len(centro) != 0:
				dif_x = media[0]-centro[0]
				dif_y = media[1]-centro[1]
				vel = Movimento(dif_x)
			else:
				vel = Procura()

			velocidade_saida.publish(vel)
			print(vel)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
