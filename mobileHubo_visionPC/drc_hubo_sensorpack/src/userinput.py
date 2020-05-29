#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    #pub = rospy.Publisher('/fsm_detect_object', String, queue_size=10)
    pub = rospy.Publisher('/order_msg_str', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "vitamin_water_orange" 
        hello_str = "2" 
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass