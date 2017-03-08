#!/usr/bin/env python

import rospy
import ConfigParser
from cv_bridge import CvBridge, CvBridgeError

from ColorDetector import ColorDetector
import numpy as np

from messages.msg import DetectedObject, DetectedObjectList, PointCloudObjectSeq, ColorObjectPar, DispatcherColor
from sensor_msgs.msg import Image
from std_msgs.msg import String

##### PARALLEL #####
def parallelCallback(dataWithHeader):
	
	# Get the Image from the message (DispatcherColor type)
	data = dataWithHeader.data
	
	# Get the corresponding numpy array
	bridge = CvBridge()
	im = bridge.imgmsg_to_cv2(data, desired_encoding = "passthrough")
	
	size = im.shape
	imageRGB=np.zeros(size, dtype=np.uint8)
	
	# /!\ The red and blue channels are inverted
	imageRGB[:,:,0] = im[:,:,2]
	imageRGB[:,:,1] = im[:,:,1]
	imageRGB[:,:,2] = im[:,:,0]
	
	# The object to run the detection
	cd = ColorDetector()
	
	# Run the detection
	blobs = cd.runCDParallel(imageRGB, activatedColors)
	
	# Make the list of detected objects as a message 
	coloredPointsList = ColorObjectPar()
	
	colorList = []
	xCentList = []
	yCentList = []
	nbPixList = []
	
	# Fill all these lists
	for blob in blobs:
		# The color of the object
		colorList.append(blob[0])
		
		# Its barycenter coordinates in pixels
		xCentList.append(blob[1])
		yCentList.append(blob[2])
		
		# Its number of pixels
		nbPixList.append(blob[3])
	
	coloredPointsList.colorList = colorList
	coloredPointsList.xCentList = xCentList
	coloredPointsList.yCentList = yCentList
	coloredPointsList.nbPixList = nbPixList
	# The header given in the input message is passed so the synchronization can be maintained
	coloredPointsList.header = dataWithHeader.header
	
	# Publish the list of detected objects	
	publisher.publish(coloredPointsList)	
	
##### SEQUENTIAL #####
def sequentialCallback(data):
	# Get the information from the message about the point clouds
	x = np.array(data.Xlist, dtype = "float")
	y = np.array(data.Ylist, dtype = "float")
	z = np.array(data.Zlist, dtype = "float")
	r = np.array(data.Rlist, dtype = "float")
	g = np.array(data.Glist, dtype = "float")
	b = np.array(data.Blist, dtype = "float")
	labels = np.array(data.Clusters, dtype = "float")
	
	# Rebuild the image as a numpy array
	d = np.array([x,y,z,r,g,b,labels]).transpose()

	# The object to run the detection
	cd = ColorDetector()

	# Run the detection
	blobs = cd.runCDSequential(d,activatedColors)
	
	# Make the list of detected objects as a message
	objectsList = []

	# Fill the list
	for blob in blobs:
		# The final object
		detectedObject = DetectedObject()
		
		# The barycenter coordinates of the final object
		detectedObject.xCentroid = blob[0]
		detectedObject.yCentroid = blob[1]
		detectedObject.zCentroid = blob[2]
		
		# The bounding box of the final object
		detectedObject.xMin = blob[3]
		detectedObject.xMax = blob[4]
		detectedObject.yMin = blob[5]
		detectedObject.yMax = blob[6]
		detectedObject.zMin = blob[7]
		detectedObject.zMax = blob[8]
		
		# The color of the final object
		detectedObject.color = blob[9]
		
		# Add the final object to the list of detected objects
		objectsList.append(detectedObject)
	
	# Publish the list of detected objects
	publisher.publish(objectsList)

##### MAIN #####
if __name__=='__main__':
	# Launch the node
	rospy.init_node('color_detection_server', anonymous=True)
	
	# Get the configuration file
	config = ConfigParser.ConfigParser()
	config.readfp(open('configuration/configuration.cfg'))
	
	# Get the chosen method (parallel or sequential)
	method = config.get('Parameters', 'method')
	
	# Get the activatedColors for color detection
	global activatedColors
	activatedColors = config.get('Parameters', 'activatedColors')
	activatedColors = np.array(map(int, activatedColors.split(',')))
	
	if method == 'sequential':
		# Sequential
		print "sequential"
		# Create the publisher 
		publisher = rospy.Publisher('color_detection', DetectedObjectList, queue_size=10)
		
		# Get information from point cloud detection and treat them
		rospy.Subscriber('point_cloud_detection', PointCloudObjectSeq, sequentialCallback)
		
		rospy.spin()
		#sequentialTreatment()
	else:
		#Parallel
		print "Parallel"
		
		# Create the publisher
		publisher = rospy.Publisher('color_detection', ColorObjectPar, queue_size=10) 
		
		# Get information from dispatcher and treat them
		rospy.Subscriber('dispatcher_color', DispatcherColor, parallelCallback)
		
		rospy.spin()
		#parallelTreatment()
