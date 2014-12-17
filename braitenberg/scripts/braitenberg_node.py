#!/usr/bin/env python

import rospy
from numpy import *
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, ByteMultiArray
from geometry_msg.msg import Twist

def callback(data):
    array_coefVx    = array([-8, -20, -32, -20, -8])
    array_coefOmega = array([-1, -5, +2, +5, +1])
    array_capteur = array(data.data)

    array_vx    =  array_coefVx    *  array_capteur
    array_omega =  array_coefOmega *  array_capteur

    move_cmd = Twist()     
    move_cmd.linear.x  = 64 + array_vy.sum()
    move_cmd.angular.z = 0  + array_theta.sum()

#    dataTemp = Float32MultiArray()
#    dataTemp.data = data.data + tuple([int(Vx), int(Vy), int(theta)])
    #---------------------------------------PUB	
    pub = rospy.Publisher('braitenberg/cmd', Twist) #, queue_size=10)
    pub.publish(move_cmd)

 #   pub_capt = rospy.Publisher('cmdCapteurs', Float32MultiArray)
 #   pub_capt.publish(dataTemp)
    

    
def braitenberg():

    rospy.init_node('braitenberg', anonymous=True)
    rospy.Subscriber('capteurs', Float32MultiArray, callback)
#    rospy.Subscriber('braitenberg/vx/poids', Float32MultiArray, callback)
    rospy.spin()

    r = rospy.Rate(10) # 10hz
   

    while not rospy.is_shutdown():
        r.sleep()
        
if __name__ == '__main__':
    try:
        braitenberg()
    except rospy.ROSInterruptException: pass
