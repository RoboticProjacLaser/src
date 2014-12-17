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

VERBOSE=True

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)
    self.posi_pub = rospy.Publisher("position", Int16)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback,  queue_size = 1)
    if VERBOSE :
      print 'Je suis lancer'

  def callback(self, ros_data):
    #### direct conversion to CV2 ####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    
    #(rows,cols,channels) = image_np.shape
    #image_np = image_np[0:64, 0:480] 

    #equ = cv2.equalizeHist(image_np)
    #res = np.hstack((image_np,equ))

    #### Feature detectors using CV2 #### 
    # "","Grid","Pyramid" + 
    # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF", "GridFAST"
    #method = "GridFAST"
    #feat_det = cv2.FeatureDetector_create(method)
    #time1 = time.time()

    # convert np image to grayscale
    #featPoints = feat_det.detect( cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
    #time2 = time.time()
    #if VERBOSE :
    #  print '%s detector found: %s points in: %s sec.'%(method, len(featPoints),time2-time1)

    #for featpoint in featPoints:
    #  x,y = featpoint.pt
    #  cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)

    #### Create CompressedIamge ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    # Publish new image
    self.image_pub.publish(msg)


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
