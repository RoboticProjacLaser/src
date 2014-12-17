#!/usr/bin/env python
import roslib
roslib.load_manifest('detect_line')
import sys
import rospy
import cv2,cv
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
    self.image_pub  = rospy.Publisher("/output/image_raw/compressed",  CompressedImage )
 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback,  queue_size = 1)
    if VERBOSE :
      print 'Je suis lancer'

  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~[CALLBACK]
  def callback(self, ros_data):
   #print 'Hello'
	# Recuperation video
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
	
	
    time1 = time.time()
	# On isole une zone
    roi = image_np[10:90,0:640]
    roiImg = np.array(roi)
    roi = cv.fromarray(roi)	
	
	# On enleve le bruit
    #print type(roiImg)
    cv2.bitwise_not(roiImg, roiImg);
    erodeElmt = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dilateElmt = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    roiImg = cv2.erode(roiImg, erodeElmt, 1)
    roiImg = cv2.dilate(roiImg, dilateElmt, 1)
    roiImg = cv2.cvtColor(roiImg,cv2.COLOR_BGR2GRAY)
    # On cherche les contours de la ligne
    ret,thresh = cv2.threshold(roiImg,127,255,cv2.THRESH_BINARY)
    #print type(thresh)
    contours,hierarchy = cv2.findContours(thresh, 1, 2)
	
	# On cherche le centre de la zone
    cnt = contours[0]
    M = cv2.moments(cnt)
    area = cv2.contourArea(cnt)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
	
    time2 = time.time()
    if VERBOSE :
      print 'Mesure faite en %s.'%(time2-time1)
	
    cv.Circle(roi, (cx,cy),2 ,(0,255,0),-1)   
    #print cx
    #print cy
    #cv2.imshow('roi_img',roiImg)
	
	
	
	
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg',image_np)[1]).tostring()
    self.image_pub.publish(msg)
    
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
