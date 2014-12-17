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
 
    self.tab_capt_futur = []
    self.tab_capt_actuel = []	
	
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback,  queue_size = 1)
	
    n_cap = 12
    pas_intercap = 640/(n_cap+1)
	
    print pas_intercap
	
    cap_futur  = 3
    cap_actuel = 470
    size_cap = 5
	
    for i in range(0, n_cap+1): 	
        self.tab_capt_futur.append([i*pas_intercap,i*pas_intercap+size_cap, cap_futur, cap_futur+size_cap])
        self.tab_capt_actuel.append([i*pas_intercap,i*pas_intercap+size_cap, cap_futur, cap_actuel+size_cap])
	
    print self.tab_capt_futur[0]	
    print self.tab_capt_futur[1]
    print self.tab_capt_futur[2]
    print self.tab_capt_futur[12]	
	
    if VERBOSE :
      print 'Je suis lancer'

  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~[CALLBACK]
  def callback(self, ros_data):
  
    n_cap = 12
    pas_interCap = 640/(n_cap+1)
    cap_futur  = 3
    cap_actuel = 470
    size_cap = 5
  
  
    #--------------------------------------------[SUB]
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    image_np = cv2.cvtColor(image_np,cv2.COLOR_BGR2GRAY)
	
	# Noir et blanc + erode --------------------------
    erodeElmt = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dilateElmt = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    image_np = cv2.erode(image_np, erodeElmt, 1)
    image_np = cv2.dilate(image_np, dilateElmt, 1)
	
    #--------------------------------------------[CROP]
    (rows,cols) = image_np.shape
    #(rows,cols,channels) = image_np.shape
    #image_np = image_np[0:64, 0:480] 
    #print " ---- ", rows = 480, ", ", cols=640
 
    #--------------------------------------------[PARAM]
    val_cap_futur  = []
    val_cap_actuel = []
    val_bin_futur  = []
    val_bin_actuel = []
    #--------------------------------------------[HIST]
    #equ = cv2.equalizeHist(image_np)
    #res = np.hstack((image_np,equ))

    #--------------------------------------------[PROCESS]
    time1 = time.time()

    for i in range(0, n_cap):
        #print "capteur : ", i
        moy4i_futur  = 0
        moy4i_actuel = 0
		
		# Calcul somme
		#capt = image_np[cap_futur:cap_futur + size_cap , pas_interCap*(i+1)+ size_cap][0]
	    #moy_actuel = capt.sum()
		
        for sub_i in range(0, size_cap):
            for sub_j in range(0, size_cap):
                moy4i_futur  = image_np[cap_futur +sub_j, pas_interCap*(i+1)+sub_i]+ moy4i_futur
                moy4i_actuel = image_np[cap_actuel+sub_j, pas_interCap*(i+1)+sub_i]+ moy4i_actuel
        val_cap_futur.append(  moy4i_futur )
        val_cap_actuel.append( moy4i_actuel )

    moy_futur  = self.avg(val_cap_futur  )
    moy_actuel = self.avg(val_cap_actuel )


    for i in range(0, n_cap):
        val_bin_futur.append(   int(val_cap_futur[i] >moy_futur)  )
        val_bin_actuel.append(  int(val_cap_actuel[i]>moy_actuel) )
   

    time2 = time.time()
    if VERBOSE :
      print 'Mesure faite en %s.'%(time2-time1)

    #--------------------------------------------[PUB]
    cap_futur  = Int32MultiArray()
    cap_futur.data = val_bin_futur
    self.pub_futur.publish(cap_futur)

    cap_actuel = Int32MultiArray()
    cap_actuel.data = val_bin_actuel
    self.pub_actuel.publish(cap_actuel)
  
  def avg(self, l):
    return sum(l, 0.0)/len(l)

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
