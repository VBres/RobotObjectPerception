#!/usr/bin/env python

import rospy
import ConfigParser

import message_filters
from message_filters import ApproximateTimeSynchronizer

from sensor_msgs.msg import Image, PointCloud2
from messages.msg import DispatcherPointCloud, DispatcherColor

##### PARALLEL #####
def parallelCallback(pointCloudData, colorData):
	print 'dispatcher running'
	
	dispatcherColor = DispatcherColor()
	dispatcherColor.data = colorData
	# To synchronize the color detection and the point cloud detection, every message will have the point cloud stamp
	dispatcherColor.header = pointCloudData.header
	
	dispatcherPointCloud = DispatcherPointCloud()
	dispatcherPointCloud.pointCloud = pointCloudData
	# To synchronize the color detection and the point cloud detection, every message will have the point cloud stamp
	dispatcherPointCloud.header = pointCloudData.header
	
	colorPublisher.publish(dispatcherColor)
	pointCloudPublisher.publish(dispatcherPointCloud)
	
##### SEQUENTIAL #####	
def sequentialCallback(pointCloudData):
	print 'dispatcher running'
	pointCloudPublisher.publish(pointCloudData)
	
##### MAIN ######		
if __name__=='__main__':
	# Launch the node
	rospy.init_node('dispatcher', anonymous=True)
	
	# Get the configuration file
	config = ConfigParser.ConfigParser()
	config.readfp(open('configuration/configuration.cfg'))
	
	# Get the chosen method (parallel or sequential)
	method = config.get('Parameters', 'method')
	
	if method == 'sequential':
		# Sequential
		print "sequential"
		
		# Get the topic for acquiring the point clouds from the depth camera
		pointCloudTopic = config.get('Feed', 'pclxyzrgb')
		
		# Create the publisher
		pointCloudPublisher = rospy.Publisher('dispatcher_point_cloud', PointCloud2, queue_size=10)
		
		# Get the point clouds from the depth camera
		rospy.Subscriber(pointCloudTopic, PointCloud2, sequentialCallback)
		
		rospy.spin()
	else:
		#Parallel
		print "Parallel"
		
		# Create the publisher to send informations to color detection and point cloud detection
		colorPublisher = rospy.Publisher('dispatcher_color', DispatcherColor, queue_size=10)
		pointCloudPublisher = rospy.Publisher('dispatcher_point_cloud', DispatcherPointCloud, queue_size=10)
		
		# Get the topics for acquiring the point clouds and the rgb image from the two cameras
		pointCloudTopic = config.get('Feed', 'pclxyzrgb')
		colorTopic = config.get('Feed', 'rgb')
		
		# Synchronize the two cameras
		synchronizer = ApproximateTimeSynchronizer([message_filters.Subscriber(pointCloudTopic, PointCloud2), message_filters.Subscriber(colorTopic, Image)],25, 0.1)
		synchronizer.registerCallback(parallelCallback)
		
		rospy.spin()
