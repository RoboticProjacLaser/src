#!/usr/bin/env python

import rospy
from cmdMoteur import cmdMoteur
from std_msgs.msg import String

def callback(data):
    global myCmdMoteur
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    cmd = data.data.split(' ')
    Vx = int(cmd[0])
    Vy = int(cmd[1])
    theta = int(cmd[2])
    rospy.loginfo("Vx : %d\tVy : %d\tTheta : %d "%(Vx, Vy, theta) )
    myCmdMoteur.envoiCommande(Vx,Vy,theta)

def cmdMoteur_node():
    global myCmdMoteur
    myCmdMoteur = cmdMoteur()
    rospy.init_node('cmdMoteur_node', anonymous=True)
    rospy.Subscriber("cmd", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    cmdMoteur_node()
