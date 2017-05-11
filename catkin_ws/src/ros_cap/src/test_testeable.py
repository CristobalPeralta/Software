#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
base_pub = None


def main():
    global base_pub
    rospy.init_node('test_testeable')
    rospy.loginfo('test_testeable')
# Subscriber for joint states
    base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    sub = rospy.Subscriber('/duckiebot/joy', Joy, process_callback)
    rospy.spin()


def process_callback(msj):
    msg = Twist2DStamped()
    msg.header.stamp = rospy.get_rostime()     
    msg.omega = msj.axes[0]
    msg.v = msj.axes[1]
    base_pub.publish(msg)


if __name__ == '__main__':
    main()

