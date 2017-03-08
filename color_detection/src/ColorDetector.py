#from PIL import Image
import numpy as np

from rgb2hsv import *
from findColor import *
from annex import *
from detection import *
#from visualization import *

from skimage.segmentation import slic

class ColorDetector:
	
##### PARALLEL #####
	def runCDParallel(self, rgbImage, activatedColors):

		# Get the image as an array
		data = np.asarray(rgbImage, dtype="float")

		# Get the size of the image
		size = data.shape
		
		# Reduce the size of the image
		data = pooling(data,size)
		size = data.shape
		nbRow, nbCol, nbCha = size
		
		# Normalized the image to get a better segmentation
		dataN = normalizeImageRGB(np.reshape(data,(-1,3)))
		dataN = np.reshape(dataN,(nbRow, nbCol, nbCha))

		# Segment the image
		clusters = slic(dataN, n_segments = 500, compactness = 100, sigma=1)
		clusters = np.reshape(clusters,(nbRow*nbCol))
		nbClusters = len(np.unique(clusters))

		data = np.reshape(data,(-1,3))
		
		## TO DECOMMENT FOR TESTING AND DEBUGING
		#segmentedImage = segmentationVisualization(clusters, size)
		#segmentedImage.save('segmentedImage.png')	# Possible saving of the segmented image
		#segmentedImage.show()
		
		# Detect objects
		blobs = detection(data, size, clusters, nbClusters, activatedColors)
		
		## TO DECOMMENT FOR TESTING AND DEBUGING
		#visualizeResults(blobs, size)
	    #imageResult.save('resultColor.png')
		#imageResult.show()	
		
		# Construct founded objects
		allObjs = []
		for j in range(len(blobs)):
			rows, cols = ind2sub(np.array([nbRow, nbCol]), np.array(blobs[j][1]))
			rows = np.asarray(sorted(rows))
			cols = np.asarray(sorted(cols))

			yBarycenter = rows[rows.size/2]*2 # *2 because we have the pooling, remove if not
			xBarycenter = cols[cols.size/2]*2 # *2 because we have the pooling, remove if not
			
			color = blobs[j][0]
			nbPixBlob = len(blobs[j][1])
			
			obj = [color, xBarycenter, yBarycenter, nbPixBlob]
			allObjs = allObjs + [obj]
		
		print "number of detected objects : ",len(blobs)
		print allObjs
		
		return allObjs
		
##### SEQUENTIAL #####

	def runCDSequential(self, data, activatedColors):
	
		# Get the number of point cloud
		nbBlobs = np.max(data[:,6].astype("int32")) + 1
		
		allObjs = []
		#For each point cloud, detect if there is several objects in
		for i in range(nbBlobs):
		    
		    # Get the information of the point cloud i
			index = (data[:,6] == i)
			argIndex = np.argwhere(data[:,6] == i).transpose()
			argIndex = argIndex[0]
			currentBlob = data[index,:]
			
			# Normalized to get a better segmentation
			currentBlobRGBN = normalizeImageRGB(currentBlob[:,3:6])
			
			# Segment the point cloud by its color
			clusters, nbClusters = segmentation(currentBlobRGBN)
			
			# Detect objects
			blobs = afterPCDetection(currentBlob, clusters, nbClusters, activatedColors, argIndex)
			
			## TO DECOMMENT FOR TESTING AND DEBUGING
			#visualizeResults(blobs, size)
			#imageResult.save('resultColor.png')
	    	#imageResult.show()	
			
			# Construct founded objects
			for j in range(len(blobs)):
				rows = np.asarray(sorted(data[blobs[j][1],0]))
				cols = np.asarray(sorted(data[blobs[j][1],1]))
				depths = np.asarray(sorted(data[blobs[j][1],2]))
				
				xBarycenter = rows[rows.size/2]
				xMin = rows[0]
				xMax = rows[rows.size-1]
				
				yBarycenter = cols[cols.size/2]
				yMin = cols[0]
				yMax = cols[cols.size-1]
				
				zBarycenter = depths[depths.size/2]
				zMin = depths[0]
				zMax = depths[depths.size-1]
				
				color = blobs[j][0]
				
				obj = [xBarycenter, yBarycenter, zBarycenter, xMin, xMax, yMin, yMax, zMin, zMax, color]
				allObjs = allObjs + [obj]
				
		
		print allObjs
		return allObjs

