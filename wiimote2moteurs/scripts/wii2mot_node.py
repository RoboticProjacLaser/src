#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from cmdMoteur.msg import Cmd


def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.axes)
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("joy", Joy, callback)

    rospy.spin()
        
if __name__ == '__main__':
    listener()
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talkerCmd():
    pub = rospy.Publisher('cmd', String) #, queue_size=10)
    rospy.init_node('talkerCmd', anonymous=True)
    r = rospy.Rate(10) # 10hz
   
    # Tableau des valeurs de la wiimote avec data 
    un = data.buttons[0]
    deux = data.buttons[1]
    A = data.buttons[2]
    B = data.buttons[3]
    Plus = data.buttons[4]
    Moins = data.buttons[5]
    Haut = data.buttons[7]
    Bas = data.buttons[6]
    Gauche = data.buttons[8]
    Droite = data.buttons[9]
    Home = data.buttons[10]

    sens = 1
    while not rospy.is_shutdown():
	#Vx = -4*(120 - data.axes[1][cwiid.Y])
        #Vy = -4*(120 - data.axes[1][cwiid.X])

 	# Cast basse valeur, evite les oscillations
            if  A:    # A botton
		print "A"
		flag = 1-flag 
		Vx = 0
		Vy = 0
		theta = 0
            if  B:    # B together
		print "B"
            if  Haut:  # Up botton
		print "UP"
		Vy+=5
            if  Gauche:  # Left botton
		print "LEFT"
		Vx+=5
            if  Droite:  # Right botton
		print "RIGHT"
		Vx-=5
            if  Bas:  # Down botton
		print "DOWN"
		Vy-=5
            if  Moins:   # Minus botton
		print "-"
		theta-=5
            if  Plus: # Plus botton
		print "+"
		theta+=5
            if  Home:   # home botton
		print "HOME"
            if  Un:   # 1 botton
		#theta-=5
            if  Deux:   # 2 botton
		#theta+=5
	    #if theta >
#		theta=5
#	    if theta <
#		theta=-5
	    #Commandes moteurs
	    #print Vx, " ", Vy, " ", theta

	#Cast des valeurs
	if Vx > 127:
	    Vx = 127
	elif Vx < (-127):
	    Vx = -127
	if Vy > 127:
	    Vy = 127
	elif Vy <(-127):
	    Vy = -127

	if flag == 0: 
            str = 0+"_"+0+"_"+0
		
	#print Vx," ", Vy, " ", theta
        #ser.write(miseEnForme.miseEnForme(Vx,Vy,theta))
	
        str = %Vx+"_"+%Vy+"_"+theta
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()
        
if __name__ == '__main__':
    try:
        talkerCmd()
    except rospy.ROSInterruptException: pass
