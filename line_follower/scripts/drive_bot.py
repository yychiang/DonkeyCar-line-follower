#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

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
  lower_yellow = numpy.array([ 200, 200, 190])
  upper_yellow = numpy.array([255, 255, 255])
  mask = cv2.inRange(cv_image, lower_yellow, upper_yellow)
  
  h, w, d = cv_image.shape
  search_top = int(3*h/4)
  search_bot = int(3*h/4 + 20)
  
  mask[0:search_top, 0:w] = 0
  mask[search_bot:h, 0:w] = 0
  M = cv2.moments(mask)
  if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      # linear_x = 0.67508
      linear_x = 0
      angular_z = -float(err) / 100

      drive_bot(linear_x, angular_z)
      # END CONTROL
  
  ####################################################################    
  
  cv2.imshow("Image window", cv_image)
  cv2.imwrite("image.jpg",cv_image)

  cv2.waitKey(3)
    
    
def callback(cmd_msg):
    motor_command = Twist()
    motor_command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('drive_to_target')
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
    motor_command.linear.x = req.linear_x
    motor_command.linear.y = 0
    motor_command.linear.z = 0
    motor_command.angular.x = 0
    motor_command.angular.y = 0
    motor_command.angular.z = req.angular_z
    
    motor_command_publisher.publish(motor_command)
    s = "linear: "+str(motor_command.linear.x)+" "+"angular: "+ str(motor_command.angular.z)
    return s
    
def drive_to_target():
    vel_msg = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('drive_to_target', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    image_sub = rospy.Subscriber("/raspicam_node/image",Image, process_image_callback)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    drive_to_target()
