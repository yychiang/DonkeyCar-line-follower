#!/usr/bin/env python

import rospy
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

import time
from geometry_msgs.msg import Twist


global prev_cte
prev_cte=0

bridge = CvBridge()



def process_image_callback(ros_image):
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  ####################################################################
  hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  lower_color = numpy.array([90, 140, 95])
  upper_color = numpy.array([110, 160, 115])
  mask = cv2.inRange(hsv, lower_color, upper_color)
  cv2.imshow("Image window", mask)
  
  
  h, w, d = cv_image.shape
  search_top = int(3*h/4)
  search_bot = int(3*h/4 + 20)
  
  mask[0:search_top, 0:w] = 0
  mask[search_bot:h, 0:w] = 0
  M = cv2.moments(mask)
  global prev_cte

  if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(cv_image, (cx, cy), 8, (0,0,255), -1)
      # BEGIN CONTROL
      cte = cx - w/2
      diff_cte = cte - prev_cte
      prev_cte = cte
      angular_z = - 0.08 * cte - 1.2 * diff_cte
      linear_x = 1.8
      
      vel_msg = Twist()
      
      pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
      
      # rate = rospy.Rate(10) # 10hz
    
      vel_msg.angular.z = angular_z
      vel_msg.linear.x  = linear_x
      print(vel_msg.angular.z)
      pub.publish(vel_msg)
      
      # while not rospy.is_shutdown():
      #    pub.publish(vel_msg)

  # END CONTROL ####################################################################    
  
  cv2.imshow("Image window", mask)
  cv2.waitKey(3)

  
def main(args):
  rospy.init_node('process_image')
  rospy.Subscriber("/camera/rgb/image_raw",Image, process_image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
