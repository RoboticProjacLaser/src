#!/usr/bin/env python
import roslib
roslib.load_manifest('detect_line')
import sys
import rospy
import cv2
import numpy as np
import time

from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32MultiArray

VERBOSE=True

#================================================[PROCESS]
class image_converter:

  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~[INIT]
  def __init__(self):
    self.posi_pub = rospy.Publisher("position", Int16)
    self.pub_futur  = rospy.Publisher('/ligne/capteurs_futur',  Int32MultiArray )
    self.pub_actuel = rospy.Publisher('/ligne/capteurs_actuel', Int32MultiArray )
    self.image_modified = rospy.Publisher('/camera/modified/image/compressed', CompressedImage )
 
    self.tab_cap_futur = []
    self.tab_cap_actuel = []	
	
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback,  queue_size = 1)
	
   #Param indice capts
    self.n_cap = 12
    pas_intercap = 640/(self.n_cap+1)
    self.cap_futur  = 3
    self.cap_actuel = 470
    self.size_cap = 5
	
    #Creation indice
    for i in range(1, self.n_cap+2): 	
        self.tab_cap_futur.append(i*pas_intercap)
        self.tab_cap_futur.append(i*pas_intercap+self.size_cap)	
        self.tab_cap_actuel.append(i*pas_intercap)
        self.tab_cap_actuel.append(i*pas_intercap+self.size_cap)
	
    if VERBOSE :
      print 'Je suis lancer'

  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~[CALLBACK]
  def callback(self, ros_data):
  
    
    #--------------------------------------------[SUB]
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
	
    time1 = time.time()
	# Noir et blanc + erode --------------------------
    image_np = cv2.cvtColor(image_np,cv2.COLOR_BGR2GRAY)
    #erodeElmt = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    #dilateElmt = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    #image_np = cv2.erode(image_np, erodeElmt, 1)
    #image_np = cv2.dilate(image_np, dilateElmt, 1)
    		 
    #--------------------------------------------[PARAM]
    moy_capt_futur = []
    moy_capt_actuel = []
    #--------------------------------------------[PROCESS]
    

    for i in range(0, (self.n_cap*2),2):
		# Calcul moyenne pixels chaque capteur
        moy_capt_futur.append(int( (image_np[self.cap_futur:self.cap_futur+self.size_cap, self.tab_cap_futur[i]:self.tab_cap_futur[i+1]]).mean() ))
        #cv2.rectangle(image_np,(self.tab_cap_futur[i],self.cap_futur),(self.tab_cap_futur[i+1],self.cap_futur+self.size_cap),(255,0,0),1)
        moy_capt_actuel.append(int( (image_np[self.cap_actuel:self.cap_actuel+self.size_cap, self.tab_cap_actuel[i]:self.tab_cap_actuel[i+1]]).mean() ))
        #cv2.rectangle(image_np,(self.tab_cap_actuel[i],self.cap_actuel),(self.tab_cap_actuel[i+1],self.cap_actuel+self.size_cap),(255,0,0),1)
    # Moyenne et ecart type des capteurs	
    moy_ligne_actuel = int(np.mean(moy_capt_actuel))
    ect_ligne_actuel = int(np.std(moy_capt_actuel))
    moy_ligne_futur = int(np.mean(moy_capt_futur))
    ect_ligne_futur = int(np.std(moy_capt_futur))
	
    print ect_ligne_actuel
    print ect_ligne_futur
	
    #On met a 0 les valeurs vu en blanc / 1 en noir
    for i in range(0, self.n_cap):
        if(moy_capt_futur[i] <= moy_ligne_futur and ect_ligne_futur > 30):
            moy_capt_futur[i] = 1;
        else: 
            moy_capt_futur[i] = 0;
        if(moy_capt_actuel[i] <= moy_ligne_actuel and ect_ligne_actuel > 30):
            moy_capt_actuel[i] = 1;
        else:
            moy_capt_actuel[i] = 0;
			
    time2 = time.time()
    if VERBOSE:
     print 'Mesure faite en %s.'%(time2-time1)

    #--------------------------------------------[PUB]
    cap_futur  = Int32MultiArray()
    cap_futur.data = moy_capt_futur
    cap_actuel = Int32MultiArray()
    cap_actuel.data = moy_capt_actuel
	
     #Publisher
    #msg = CompressedImage()
    #msg.header.stamp = rospy.Time.now()
    #msg.format = "jpeg"
    #msg.data = np.array(cv2.imencode('.jpg',image_np)[1]).tostring()
    #self.image_modified.publish(msg)
    self.pub_futur.publish(cap_futur)
    self.pub_actuel.publish(cap_actuel)
#================================================[MAIN]
def main(args):
  ic = image_converter()
  rospy.init_node('detect_line', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
