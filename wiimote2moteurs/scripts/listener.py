#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.axes)
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("joy", Joy, callback)

    rospy.spin()
        
if __name__ == '__main__':
    listener()
