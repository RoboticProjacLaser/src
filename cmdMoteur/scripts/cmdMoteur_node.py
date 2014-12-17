#!/usr/bin/env python

import rospy
from cmdMoteur import cmdMoteur
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def callback(data):
    global myCmdMoteur
    Vx = int(data.linear.x)
    Omega = int(data.angular.z)
    rospy.loginfo("Vx : %d\tOmega : %d "%(Vx, Omega) )
    myCmdMoteur.Publish(Vx, Omega)

def cmdMoteur_node():
    global myCmdMoteur
    myCmdMoteur = cmdMoteur()
    rospy.init_node('cmdMoteur_node', anonymous=True)
    rospy.Subscriber("cmd", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    cmdMoteur_node()
