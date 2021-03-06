#!/usr/bin/env python


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# define range of blue color in HSV
lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_red = np.array([0,0,0])
upper_red = np.array([0,0,0])
lower_yellow = np.array([20,150,150])
upper_yellow = np.array([50,255,255])


class BlobColor():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self._process_image)

        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        self.min_area = 10
        self.base_pub = rospy.Publisher("/duckiebot/blob_color/image/raw", Image, queue_size=1)
        self.base_pub2 = rospy.Publisher("/duckiebot/blob_color2/image/raw", Image, queue_size=1)


    def _process_image(self,img):
        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        #Se deja en frame la imagen actual
        frame = self.cv_image
        lineas = self.cv_image2
        #Cambiar tipo de color de BGR a HSV
        image_out = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        # Filtrar colores de la imagen en el rango utilizando 
        mask = cv2.inRange(image_out, lower_yellow, upper_yellow)
        

        # Bitwise-AND mask and original image
        segment_image = cv2.bitwise_and(frame,frame, mask= mask)
        # segment_image = cv2.bitwise_and(frame,frame, mask= mask)


        kernel = np.ones((5,5),np.uint8)

        #Operacion morfologica erode
        img_out = cv2.erode(mask, kernel, iterations = 1)
        
        #Operacion morfologica dilate
        img_out = cv2.dilate(img_out, kernel, iterations = 1)

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
            if (rect[1][0] > 7 and rect[1][1] > 7 and rect[1][0] < 1.6*rect[1][1] and rect[1][1] < 1.6*rect[1][0] and rect[0][1] < 200 and rect[0][1] > 120):
                print rect
                #Dibujar un rectangulo en la imagen
                cv2.drawContours(frame, [box], 0, (0,0,0), 2)
            cv2.drawContours(lineas, [box], 0, (0,0,0), 2)
        #Publicar frame
        #image_out = cv2.cvtColor(img_out,cv2.COLOR_HSV2BGR)
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        msj = self.bridge.cv2_to_imgmsg(lineas,"bgr8")

        #Publicar Point center de mayor tamanio
        self.base_pub.publish(msg)
        self.base_pub2.publish(msj)


def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()
