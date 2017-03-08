from PIL import Image
import numpy as np

# Function to help visualize the result of the color segmentation
		# INPUT : (clusters, 1) labels of the cluster from the segmentation
		#		  (nbRow, nbCol, nbCha) size, size of the image
		# OUTPUT : imageSegmented, an image of Image type that we can save and show
def segmentationVisualization(clusters, size):
	nbRow, nbCol, nbCha = size
	nbPix = nbRow * nbCol
	
	# Segmentation visualization
	regions = np.array([clusters, clusters, clusters]).transpose()
	for i in range(nbPix) :
		if regions[i][0] == 0:		    	# The first area is colored in blue
			regions[i] = [0, 0, 255]
		elif regions[i][0] == 1:			# The second area is colored in green
			regions[i] = [0, 255, 0]
		elif regions[i][0] == 2:			# The third area is colored in red
			regions[i] = [255, 0 ,0]
		elif regions[i][0] == 3:			# The fourth area is colored in yellow
			regions[i] = [255, 255,0]
		elif regions[i][0] == 4:			# The fifth area is colored in light blue
			regions[i] = [0, 255, 255]
		elif regions[i][0] == 5:			# The sixth area is colored in pink
			regions[i] = [255, 0, 255]
		elif regions[i][0] == 6:			# The seventh area is colored in white
			regions[i] = [255, 255, 255]
										# If there is more than 7 areas, the last ones would be in black
 								
 	# Rebuild the image											
 	imageSegmented = regions.reshape(nbRow,nbCol,nbCha)
 	imageSegmented = np.array(imageSegmented)
	imageSegmented = np.uint8(imageSegmented)
	imageSegmented = Image.fromarray(imageSegmented)
	
	return imageSegmented

# Function to help visualize the result of the object detection, it shows a black image where the detected objects appear with their color
		# INPUT : [[color, indexes of blobs], ... ] blobs, detected blobs
		#		  (nbRow, nbCol, nbCha) size, size of the image
		# OUTPUT : imageResult, an image of Image type that we can save and show
def visualizeResults(blobs, size):
	nbRow, nbCol, nbCha = size
	nbPix = nbRow * nbCol
	
	imageResult = np.zeros([nbPix,nbCha])
	
	for i in range(len(blobs)):
		if blobs[i][0] == 0:
			imageResult[blobs[i][1]] = [255, 255, 0]	#yellow
		elif blobs[i][0] == 1:
			imageResult[blobs[i][1]] = [255, 0, 0] 		#red
		elif blobs[i][0] == 2:
			imageResult[blobs[i][1]] = [0, 255, 0]		#green
		elif blobs[i][0] == 3:
			imageResult[blobs[i][1]] = [0, 0, 255]		#blue
		elif blobs[i][0] == 4:
			imageResult[blobs[i][1]] = [255, 0, 255]	#purple
		elif blobs[i][0] == 5:
			imageResult[blobs[i][1]] = [255, 125, 0]	#orange
	
	# Rebuild the image	
	imageResult = imageResult.reshape(nbRow,nbCol,nbCha)
	imageResult = np.uint8(imageResult)
	imageResult = Image.fromarray(imageResult)
	
	return imageResult
