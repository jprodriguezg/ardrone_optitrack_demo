#!/usr/bin/env python

from visual_object_detector.srv import *
import rospy
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

image_data=None
cv_bridge=None

# TODO: complete
def detect_object(cv_frame):
    """Return 0 if no object were found, or a positive integer <class> if an object
    of class <class> was found."""

	imghsv = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2HSV)

	# define range of red color in HSV
	lower_red = np.array([0, 200,0])
	upper_red = np.array([10,255,200])

	# Threshold the HSV image to get only red colors
	mask1 = cv2.inRange(imghsv, lower_red, upper_red)

	lower_red = np.array([169, 200,0])
	upper_red = np.array([179, 255,200])

	# Threshold the HSV image to get only red colors
	mask2 = cv2.inRange(imghsv, lower_red, upper_red)

	mask = cv2.bitwise_or(mask1, mask2);
	nz = float(cv2.countNonZero(mask))/mask.size;
	det = nz > 0.004;
	print '', nz, ' ', det
	if(det)
		return 1
	else
		return 0

def handle_detect_object(req):
    """Return a response with value -1 if an error occured else the value returned by the object detector (a positive integer)"""
    try:
        cv_frame = cv_bridge.imgmsg_to_cv2(image_data)
        detection=detect_object(cv_frame)
        return DetectObjectResponse(detection)
    except Exception as e:
        rospy.logerr("Could not complete the request because of exception %s",e)
        return DetectObjectResponse(-1)

def new_frame(data):
    global image_data
    image_data=data

def main():
    rospy.init_node('detect_object_node')
    service = rospy.Service('detect_object', DetectObject, handle_detect_object)
    subscriber=rospy.Subscriber('input_image',Image,new_frame,queue_size=1)
    global cv_bridge    
    cv_bridge = CvBridge()
    print "Ready to detect object with ",cv_bridge
    rospy.spin()

if __name__ == "__main__":
    main()
