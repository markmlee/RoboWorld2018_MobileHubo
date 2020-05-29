#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker_done():
    pub = rospy.Publisher('/done_input', String, queue_size=10)
    rospy.init_node('talker_done', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "done" 
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker_done()
    except rospy.ROSInterruptException:
        pass