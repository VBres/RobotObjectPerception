#!/usr/bin/env python

import rospy
import math
import numpy as np
import message_filters
from messages.msg import DetectedObject, DetectedObjectList, PointCloudObjectSeq, ColorObjectPar, PointCloudObjectPar
import ConfigParser
import sys
sys.path.append('color_detection/src') ## TODO
from findColor import findColorRangeHSV

##### PARALLEL #####
def parallelCallback(pointCloudData, colorData):
	print "merger running"
	
	# Get point cloud detected objects
	xCenterList = np.array(pointCloudData.Xlist, dtype = "float")
	yCenterList = np.array(pointCloudData.Ylist, dtype = "float")
	zCenterList = np.array(pointCloudData.Zlist, dtype = "float")
	xMinList = np.array(pointCloudData.MinX, dtype = "float")
	xMaxList = np.array(pointCloudData.MaxX, dtype = "float")
	yMinList = np.array(pointCloudData.MinY, dtype = "float")
	yMaxList = np.array(pointCloudData.MaxY, dtype = "float")
	zMinList = np.array(pointCloudData.MinZ, dtype = "float")
	zMaxList = np.array(pointCloudData.MaxZ, dtype = "float")
	rList = np.array(pointCloudData.RList, dtype = "int")
	gList = np.array(pointCloudData.GList, dtype = "int")
	bList = np.array(pointCloudData.BList, dtype = "int")
	
	# Get color detected objects
	colors = np.array(colorData.colorList, dtype = "int")
	xCentroidList = np.array(colorData.xCentList, dtype = "float")
	yCentroidList = np.array(colorData.yCentList, dtype = "float")
	nbPixList = np.array(colorData.nbPixList, dtype = "int")
	
	# The list that will contain the detected objects after the merging
	mergedObjects = []
	
	# For each point cloud detected object corresponds the associated color detected object
	# It is -1 if there is no match (meaning there is no color detected object with the same color as the point cloud detected object) 
	associatedObjects = np.ones(len(xCenterList)) * (-1)
	
	# For each point cloud detected object, look if it matches a color detected object
	nbPointCloudObjects = len(xCenterList)
	for i in range(nbPointCloudObjects):
		
		# Get the information about the current point cloud detected object
		xPC = xCenterList[i]
		yPC = yCenterList[i]
		zPC = zCenterList[i]
		xMinPC = xMinList[i]
		xMaxPC = xMaxList[i] 
		yMinPC = yMinList[i]
		yMaxPC = yMaxList[i] 
		zMinPC = zMinList[i]
		zMaxPC = zMaxList[i] 
		r = rList[i]
		g = gList[i]
		b = bList[i]
		
		# Projection of the 3D points in depth reference in 2D points  
		line1 = np.array(map(float, PFirstLine.split(',')))
		line2 = np.array(map(float, PSecondLine.split(',')))
		line3 = np.array(map(float, PThirdLine.split(',')))

		P = np.array([line1,line2,line3])
		coord3D = np.array([xPC, yPC, zPC, 1]).transpose()
		coord2D = np.dot(P,coord3D)
		
		xPix = coord2D[0] / coord2D[2]
		yPix = coord2D[1] / coord2D[2]

		# Find the color of the current point cloud
		colorPC = findColorRangeHSV(np.array([r,g,b], dtype = "float"),activatedColors)
		
		# If it is not a black or white object, find the closest color detected object of the same color
		if colorPC != -1:
			minDist = float("infinity")
			for j in range(len(colors)):
				if colors[j] == colorPC:
					# The distance between the color barycenter and the point cloud barycenter
					currentDist = math.sqrt((xCentroidList[j] - xPix)**2 + (yCentroidList[j] - yPix)**2)
					if currentDist < minDist :
						minDist = currentDist
						associatedObjects[i] = j
	
	nbColorObjects = len(colors)
	# For each color detected object, if it corresponds to several point cloud detected objects, the latter will be merged					
	for obj in range(nbColorObjects):
		
		# If a color detected object doesn't match any point cloud detected object, it won't be kept
		if obj in associatedObjects:
			
			# The final object
			mergedObject = DetectedObject()
			
			# Get all the point cloud detected objects that match the current color detected object
			indObj = np.argwhere(associatedObjects == obj).transpose()
			indObj = indObj[0]
			
			# The barycenter coordinates of the final object
			mergedObject.xCentroid = np.mean(xCenterList[indObj])
			mergedObject.yCentroid = np.mean(yCenterList[indObj])
			mergedObject.zCentroid = np.mean(zCenterList[indObj])
			
			# The bounding box of the final object
			mergedObject.xMin = np.min(xMinList[indObj])
			mergedObject.xMax = np.max(xMaxList[indObj])
			mergedObject.yMin = np.min(yMinList[indObj])
			mergedObject.yMax = np.max(yMaxList[indObj])
			mergedObject.zMin = np.min(zMinList[indObj])
			mergedObject.zMax = np.max(zMaxList[indObj])
			
			# The color of the final object
			mergedObject.color = colors[obj]

			# Add the final object to the list of detected objects
			mergedObjects.append(mergedObject)
	
	print "number of detected objects : ", len(mergedObjects)
	print mergedObjects
	# Publish the detected objects
	publisher.publish(mergedObjects)
		
##### SEQUENTIAL #####	
def sequentialCallback(data):
	print 'merger running'
	print data
	# Publish the detected objects
	publisher.publish(data)
	
##### MAIN #####		
if __name__ == '__main__':
	# Launch the node
	rospy.init_node('merger', anonymous=True)
	
	# Create the publisher for the final results
	publisher = rospy.Publisher('object_detection', DetectedObjectList, queue_size=10)
	
	# Get the configuration file
	config = ConfigParser.ConfigParser()
	config.readfp(open('configuration/configuration.cfg'))
	
	# Get the chosen method (parallel or sequential)
	method = config.get('Parameters', 'method')
	
	# Get the projection matrix to compare point cloud detection and color detection
	PFirstLine = config.get("Parameters",'PFirstLine')
	PSecondLine = config.get("Parameters",'PSecondLine')
	PThirdLine = config.get("Parameters",'PThirdLine')
	
	# Get the activatedColors for color detection
	global activatedColors
	activatedColors = config.get('Parameters', 'activatedColors')
	activatedColors = np.array(map(int, activatedColors.split(',')))
	
	if method == 'sequential':
		# Sequential
		print "sequential"
		
		# Get the result of color detection
		rospy.Subscriber('color_detection', DetectedObjectList, sequentialCallback)
		
		rospy.spin()
	else:
		#Parallel
		print "Parallel"
		
		# Get the results from color detection and point cloud detection
		pointCloudSubscriber = message_filters.Subscriber('point_cloud_detection', PointCloudObjectPar) 
		colorSubscriber = message_filters.Subscriber('color_detection', ColorObjectPar) 
	
		# Synchronize the two results (they have to correspond to same image from the camera, meaning they would have the same (modified) stamp
		ts = message_filters.TimeSynchronizer([pointCloudSubscriber, colorSubscriber], 25)
		ts.registerCallback(parallelCallback)
	
		rospy.spin()
