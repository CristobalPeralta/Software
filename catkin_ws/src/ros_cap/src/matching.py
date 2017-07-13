#!/usr/bin/env python

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from matplotlib import pyplot as plt

class matching():

    def __init__(self, img2):


        # Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self._process_image)

        # Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        # Ultima imagen adquirida
        self.cv_image = Image()
        self.orb = cv2.ORB_create()
        self.img2 = img2
        self.kp2 = self.orb.detect(img2,None)
        self.kp2, self.des2 = self.orb.compute(self.img2, self.kp2)       
        self.base_pub = rospy.Publisher("/duckiebot/matching/image/raw", Image, queue_size=1)

        
    def _process_image(self,img):

        # Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Se deja en frame la imagen actual
        img1 = self.cv_image

        # Inicia detector orb
        
        # buscar puntos de interes

        kp1 = self.orb.detect(self.cv_image,None)
   
        # computar los descriptorescon orb
        kp1, des1 = self.orb.compute(self.cv_image,kp1)

        # imagen del pato
 
        # dkjskdjs

        # descriptores y puntos de interes de img2

        # crear BFmatcher
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # descriptores iguales =O
        print des1.shape
        print self.des2
        matches = bf.match(des1,self.des2)

        # ordenar por orden de distancia
        matches = sorted(matches, key = lambda x:x.distance)

        # dibujemos!!
        img3 = cv2.drawMatches(img1,kp1,self.img2,self.kp2,matches[:10], img1,flags=2)
        # cambiar de cv2 a ros
        msg = self.bridge.cv2_to_imgmsg(img3, "bgr8")        

        # publicar
        self.base_pub.publish(msg)
 
def main():

    rospy.init_node('matching')
    img2 = cv2.imread("patito33.png")
    
    matching(img2)

    rospy.spin()

if __name__ == '__main__':
    main()       
