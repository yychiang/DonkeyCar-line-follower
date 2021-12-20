#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist


def talker():
    vel_msg = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    vel_msg_1 = Twist()
    vel_msg_2 = Twist()
    pub1 = rospy.Publisher('cmd_vel_1', Twist, queue_size=10)
    pub2 = rospy.Publisher('cmd_vel_2', Twist, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        vel_msg.linear.y = 2.0;
        vel_msg.angular.z = 0.0;
        pub.publish(vel_msg)
        
        vel_msg_1.linear.x = 0.0;
        vel_msg_1.angular.z = -2.0;
        vel_msg_2.linear.x = 0.0;
        vel_msg_2.angular.z = 2.0;
        pub1.publish(vel_msg_1)
        pub2.publish(vel_msg_2)
            
        rate.sleep()
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
