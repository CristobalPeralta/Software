#!/usr/bin/env python

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from matplotlib import pyplot as plt

class orb():

    def __init__(self):


        # Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self._process_image)

        # Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        # Ultima imagen adquirida
        self.cv_image = Image()
        self.base_pub = rospy.Publisher("/duckiebot/orb/image/raw", Image, queue_size=1)
        


    def _process_image(self,img):

        # Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Se deja en frame la imagen actual
        frame = self.cv_image

        # Inicia detector orb
        orb = cv2.ORB_create()

        # buscar puntos de interes

        kp = orb.detect(self.cv_image,None)
   
        # computar los descriptorescon orb
        kp, des = orb.compute(self.cv_image,kp)

        #dibujar los puntos de interes
        img2 = cv2.drawKeypoints(self.cv_image, kp, None, color=(0,255,0), flags=0)
        
        # cambiar de cv2 a ros
        msg = self.bridge.cv2_to_imgmsg(img2, "bgr8")        

        # publicar
        self.base_pub.publish(msg)
 
def main():

    rospy.init_node('orb')

    orb()

    rospy.spin()

if __name__ == '__main__':
    main()       
