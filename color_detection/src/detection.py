import numpy as np
import math

from findColor import *
from annex import *

from sklearn.cluster import MeanShift, estimate_bandwidth
from sklearn.datasets.samples_generator import make_blobs
from sklearn import preprocessing


# Function to segment an image with Meanshift
	# INPUT : (nbPoints,nbFeatures) data to segment
	#	      size, the size of the image
	# OUTPUT : (nbPoints,1) clusters, a vector where each pixel is associated with its cluster
	#		   nbClusters, the number of clusters
def segmentation(data):

	# Mean-shift segmentation
	meanshift = MeanShift(bandwidth=0.01, bin_seeding=True)
	meanshift.fit(data)	# Perform clustering
	clusters = meanshift.labels_
	nbClusters = len(np.unique(clusters))

	return clusters, nbClusters

# Function that reduces the size of the given image by 3/4. It does the mean of 4 neighbours pixels.
	# INPUT : (nbRow,nbCol,nbCha) image, the given image
	#         size, the size of the image
	# OUTPUT : (nbRow/2,nbCol/2,nbCha) smallImage, the reduce image
def pooling(image, size):
	nbRow, nbCol, nbCha = size
	
	smallImage = np.zeros((nbRow/2, nbCol/2, nbCha))
	
	for i in range(nbRow/2):
		for j in range(nbCol/2):
			smallImage[i][j] = (image[i*2][j*2] + image[i*2+1][j*2] + image[i*2][j*2+1] + image[i*2+1][j*2+1]) / 4
	
	return smallImage		
	

# Function to normalize a RGB image
		# INPUT : (nbPix, 3) data, the vectorized image
		# OUTPUT : (nbPix, 3) dataNormalized, the normalized image
def normalizeImageRGB(data):
	
	# We get the three channels
	R = data[:,0]
	G = data[:,1]
	B = data[:,2]
	
	# We compute their sum
	S = R + G + B
	
	# We normalize the channels
	Rnorm = np.zeros(len(data))
	Gnorm = np.zeros(len(data))
	Bnorm = np.zeros(len(data))
	
	index = (S != 0)
	
	Rnorm[index] = np.divide(R[index],S[index])
	Gnorm[index] = np.divide(G[index],S[index])
	Bnorm[index] = np.divide(B[index],S[index])
	
	# The result's size is (nbPix, 3)
	dataNormalized = np.array([Rnorm, Gnorm, Bnorm]).transpose()
	
	return dataNormalized

# Function to detect the objects - use for parallel
	# INPUT : (nbPixels,3) data , the image to segment
	#	      size, the size of the image
	#	      (nbPixels,1) clusters, a vector where each pixel is associated with its cluster
	#		  nbClusters, the number of clusters
	#         actColor, colors that are activated
	# OUTPUT : 	blobs, a list of blobs with their color, ex: [[blue,[index of pixels]],[red,[index of pixels]]]
	#			each blob corresponds to an object
def detection(data, size, clusters, nbClusters, actColor) :
	
	nbRow, nbCol, nbCha = size
	
	blobs = []
	for i in range(nbClusters) :
	    # Find the index of the pixels in the current cluster (indCluster)
		indCluster = np.where(clusters == i)
		indCluster = np.asarray(indCluster[0])
		indCluster = indCluster.tolist()
		
		# Get the pixels of the current cluster
		region = data[indCluster]
		
		# Look for the mean color in the current cluster
		meanColor = np.mean(region, axis=0)
		
		# Determine which type of color it is
		color = findColorRangeHSV(meanColor,actColor)
		
		# If the current cluster is colored (meaning not white but one of the detected colors)
		if color != -1:
		    
			# Get all the blobs that are of this color
			coloredBlobs = getColoredBlobs(blobs, color)
			
			# Look for the limits of the current cluster, meaning its xmin xmax ymin ymax index
			rows, cols = ind2sub(np.array([nbRow, nbCol]), np.array(indCluster))
			maximaCurrentBlob = [min(rows), max(rows), min(cols), max(cols)]
			
			# The point is to determine if two (or more) same colored clusters are neighbours so part of the same object
			neighbours = []
			for j in range(len(coloredBlobs)) :
			    
				# Look for the limits of the current blob, meaning its xmin xmax ymin ymax index
				rows, cols = ind2sub(np.array([nbRow, nbCol]), np.array(blobs[coloredBlobs[j]][1]))
				maximaBlob = [min(rows), max(rows), min(cols), max(cols)]
				
				# Are the current cluster and the current blob neighbours ? 
				if areNeighbours2D(maximaCurrentBlob, maximaBlob) :
					neighbours.append(coloredBlobs[j])
					
			# Add the current cluster to the blobs list
			blobs.append([color, indCluster])
			
			# If two clusters are neighbours they belong to the same blob
			for k in range(len(neighbours)):
				blobs[-1][1] = blobs[-1][1] + blobs[neighbours[k]-k][1]
				blobs = blobs[0:neighbours[k]-k]+blobs[neighbours[k]-k+1:len(blobs)]

	return blobs

# Function to detect the objects after point cloud detection - use for sequential
	# INPUT : (nbPoints,3) data , the points to segment
	#	      (nbPixels,1) clusters, a vector where each pixel is associated with its cluster
	#		  nbClusters, the number of clusters
	#         actColor, colors that are activated
	#         argIndex, indexes of the points in the data given at the beginning (all point cloud together given by point cloud detection)
	# OUTPUT : 	blobs, a list of blobs with their color, ex: [[blue,[index of pixels]],[red,[index of pixels]]]
	#			each blob corresponds to an object
def afterPCDetection(data, clusters, nbClusters, actColor, argIndex) :
	
	blobs = []
	
	# Find the index of the pixels in the current cluster (indCluster)
	for i in range(nbClusters) :
	    # Find the index of the pixels in the current cluster (indCluster)
		indCluster = np.where(clusters == i)
		indCluster = np.asarray(indCluster[0])
		indCluster = indCluster.tolist()
		
		# Neglect too small clusters
		if len(indCluster) >= 5 :
		    
			# We get the color of the pixels of the current cluster
			region = data[indCluster,3:6]
			
			# We look for the mean color in the current cluster
			meanColor = np.mean(region, axis=0)
			
			# We determine which type of color it is
			color = findColorRangeHSV(meanColor,actColor)
			
			# If the current cluster is colored (meaning not white but one of the detected colors)
			if color != -1:
				# Get all the blobs that are of this color
				coloredBlobs = getColoredBlobs(blobs, color)
				
				# Look for the limits of the current cluster, meaning its xmin xmax ymin ymax index
				maximaCurrentBlob = [min(data[indCluster,0]), max(data[indCluster,0]), min(data[indCluster,1]), max(data[indCluster,1]), min(data[indCluster,2]), max(data[indCluster,2])]
				
				# The point is to determine if two (or more) same colored clusters are neighbours so part of the same object
				neighbours = []
				for j in range(len(coloredBlobs)) :
				    
					# We look for the limits of the current blob, meaning its xmin xmax ymin ymax index
					indBlob = blobs[coloredBlobs[j]][1]
					maximaBlob = [min(data[indBlob,0]), max(data[indBlob,0]), min(data[indBlob,1]), max(data[indBlob,1]), min(data[indBlob,2]), max(data[indBlob,2])]
					
					# Considere that there are not two objects of the same color in the point cloud
					neighbours.append(coloredBlobs[j])
					
				# We add the current cluster to the blobs list
				blobs.append([color, indCluster])
				
				# If two clusters are neighbours they belong to the same blob
				for k in range(len(neighbours)):
					blobs[-1][1] = blobs[-1][1] + blobs[neighbours[k]-k][1]
					blobs = blobs[0:neighbours[k]-k]+blobs[neighbours[k]-k+1:len(blobs)]
					
	# Give indexes of founded blob in the array of data given at the beginning
	for i in range(len(blobs)):
		blobs[i][1] = argIndex[blobs[i][1]].tolist()
	return blobs
