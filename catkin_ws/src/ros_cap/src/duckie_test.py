#!/usr/bin/env python


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image, Joy
from duckietown_msgs.msg import WheelsCmdStamped

from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# define range of blue color in HSV
lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_red = np.array([0,0,0])
upper_red = np.array([0,0,0])
lower_yellow = np.array([20,150,120])
upper_yellow = np.array([30,255,255])


class BlobColor():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self._process_image)
        self.joy = rospy.Subscriber("/joy", Joy, self._joy_callback)
        self.pub = rospy.Publisher('/duckiebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.cmd = WheelsCmdStamped()
        self.right = False
        self.left = False


        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        self.min_area = 10
        self.base_pub = rospy.Publisher("/duckiebot/blob_color/image/raw", Image, queue_size=1)


    def _joy_callback(self, msg):
        self.cmd.header.stamp = msg.header.stamp
        self.cmd.vel_left = msg.axes[1] - msg.axes[0]
        self.cmd.vel_right = msg.axes[1]  + msg.axes[0] 
        if self.right and self.left:
            self.cmd.vel_left = -0.8
            self.cmd.vel_right = 1
        elif self.right:
            self.cmd.vel_left = 1
            self.cmd.vel_right = -0.8
        elif self.left:            
            self.cmd.vel_left = 0.8
            self.cmd.vel_right = 2

        self.pub.publish(self.cmd)
    def _process_image(self,img):
        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        #Se deja en frame la imagen actual
        frame = self.cv_image

        #Cambiar tipo de color de BGR a HSV
        image_out = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        # Filtrar colores de la imagen en el rango utilizando 
        mask = cv2.inRange(image_out, lower_yellow, upper_yellow)
        

        # Bitwise-AND mask and original image
        segment_image = cv2.bitwise_and(frame,frame, mask= mask)
        # segment_image = cv2.bitwise_and(frame,frame, mask= mask)


        #kernel = np.ones((5,5),np.uint8)

        #Operacion morfologica erode
        #img_out = cv2.erode(mask, kernel, iterations = 1)
        
        #Operacion morfologica dilate
        #img_out = cv2.dilate(img_out, kernel, iterations = 1)

        contours = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours[1]:
            #Obtener rectangulo
            #print cnt
            rect = cv2.minAreaRect(cnt)
            
            box = cv2.boxPoints(rect)
            box = np.int0(box)
			
            #print box
            #rospy.loginfo("%f,%f,%f,%f",x,y,w,h)
            #Filtrar por area minima
            #if w*h > self.min_area:
            left = 0 
            right = 0
                #Dibujar un rectangulo en la imagen
            if rect[0][1]> 120 and rect[1][0]<rect[1][1]+10 and rect[1][0]+10>rect[1][1] and rect[1][0]>5 and rect[1][1]>5:
                cv2.drawContours(frame, [box], 0, (0,0,0), 2)
                if rect[0][0]>180:
                    right = 1
                    self.right = True
                elif rect[0][0]<180:
                    left = 1
                    self.left = True
            if left!=1:
                self.left = False
            if right!=1:
                self.right = False

        #Publicar frame
        #image_out = cv2.cvtColor(img_out,cv2.COLOR_HSV2BGR)
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")


        #Publicar Point center de mayor tamanio
        self.base_pub.publish(msg)


def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()
