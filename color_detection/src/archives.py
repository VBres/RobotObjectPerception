from PIL import Image
import numpy as np
import math
import time
import sys

from annex import *
from ColorDetector import *
from detection import *
from findColor import *
from rgb2hsv import *
from visualization import *



def runCDImageMS(self, imageName):
	
	actColor = np.array([1,1,1,1,0,0])
	
	# Read the RGB image
	imageRGB = Image.open(imageName)
	#imageRGB.show()
	
	# Get the image as an array
	data = np.asarray(imageRGB, dtype="float")

	# Get the size of the image
	size = data.shape
	
	# Reduce the size of the image
	data = pooling(data,size)
	size = data.shape
	nbRow, nbCol, nbCha = size
	
	# Vectorize the image
	data = np.reshape(data,(-1,3))
	
	# Normalized the image to get a better segmentation
	dataN = normalizeImageRGB(data)
	
	# Segment the image
	clusters, nbClusters = segmentation(dataN)
	## TO DECOMMENT FOR TESTING AND DEBUGING
	imageSegmented = visu_seg(clusters, size)
	imageSegmented.save('imageSeg.png')	# Possible saving of the segmented image
	#imageSegmented.show()
	
	# Detect objects
	blobs = detection(data, size, clusters, nbClusters, actColor)
	print("number of estimated objects : %d" % len(blobs))
	
	visualizeResults(blobs, size)
	allObjs = []
	for j in range(len(blobs)):
		rows, cols = ind2sub(np.array([nbRow, nbCol]), np.array(blobs[j][1]))
			rows = np.asarray(sorted(rows))
			cols = np.asarray(sorted(cols))

			xBarycentre = cols[cols.size/2]*2 # *2 because we have the pooling, remove if not
			yBarycentre = rows[rows.size/2]*2 # *2 because we have the pooling, remove if not
			color = blobs[j][0]
			nbPixBlob = len(blobs[j][1])
			
			obj = [color, xBarycentre, yBarycentre, nbPixBlob]
			allObjs = allObjs + [obj]
	return allObjs

# Function that defines if two blobs are neighbours
	# INPUT : blob1, it contains xmin xmax ymin ymax of the first blob
	#		  blob2, it contains xmin xmax ymin ymax for the second blob
	# OUTPUT : result, true if the two blobs are neighbours
def areNeighbours3D(blob1, blob2):
	xmin1 = blob1[0]
	xmax1 = blob1[1]
	ymin1 = blob1[2]
	ymax1 = blob1[3]
	zmin1 = blob1[4]
	zmax1 = blob1[5]
	
	xmin2 = blob2[0]
	xmax2 = blob2[1]
	ymin2 = blob2[2]
	ymax2 = blob2[3]
	zmin2 = blob2[4]
	zmax2 = blob2[5]
	
	cond1 = ((xmin2 <= xmin1 <= xmax2) or (xmin2 <= xmax1 <= xmax2)) and ((ymin2 <= ymin1 <= ymax2) or (ymin2 <= ymax1 <= ymax2)) and ((zmin2 <= zmin1 <= zmax2) or (zmin2 <= zmax1 <= zmax2))
	cond2 = ((xmin1 <= xmin2 <= xmax1) or (xmin1 <= xmax2 <= xmax1)) and ((ymin1 <= ymin2 <= ymax1) or (ymin1 <= ymax2 <= ymax1)) and ((zmin1 <= zmin2 <= zmax1) or (zmin1 <= zmax2 <= zmax1))

	result = cond1 or cond2
	return result

# Function that takes a color and determines in which range of defined colors it belongs
	# INPUT : color, the given color RGB
	# OUTPUT : colorFound, a number corresponding to the color found
def findColorRangeRGB(color, activatedColors):
	r, g, b = color
	
	colorFound = -1
	if (r > 150) and (g > 150) and (b < 100) and activatedColors[0]:
		# Yellow
		colorFound = 0
	elif (r - g > 42) and (r - b > 42) and activatedColors[1]:
		# Red
		colorFound = 1
	elif (r  < 130) and (g > 110) and (b < 130) and activatedColors[2]:
		# Green
		colorFound = 2
	elif (r  < 130) and (g < 130) and (b > 110) and activatedColors[3]:
		# Blue
		colorFound = 3
	elif (r  > 150) and (g < 100) and (b > 150) and activatedColors[4]:
		# Purple
		colorFound = 4
	elif (r  > 150) and (50 < g < 100) and (b < 100) and activatedColors[5]:
		# Orange
		colorFound = 5
	return colorFound

def findColorDistMin(color, activatedColors):
	seuilWhite = 100
	seuilBlack = 50
			
	#white = [255, 255, 255]
	#black = [0, 0, 0]
	yellow = [255, 255, 0]
	red = [255, 0, 0]
	green = [0, 255, 0]
	blue = [0, 0, 255]
	purple = [255, 0, 255]
	orange = [255, 125, 0]
	
	
	colors = [yellow, red, green, blue, purple, orange]#, white, black]
	
	distanceMin = math.sqrt((colors[0][0] - color[0])**2 + (colors[0][0] - color[1])**2 + (colors[0][0] - color[2])**2)
	colorFound = 0
	
	for i in range(1,len(colors)):
		distTemp = math.sqrt((colors[i][0] - color[0])**2 + (colors[i][1] - color[1])**2 + (colors[i][2] - color[2])**2)
		if (distTemp < distanceMin):
			distanceMin = distTemp
			colorFound = i
	
	if not(activatedColors[colorFound]):
		colorFound = -1
	if (colorFound == 0):
		if distanceMin > seuilWhite:
			colorFound = -1
	elif (colorFound == 2) or (colorFound == 3):
		if distanceMin > seuilBlack:
			colorFound = -1
	
	
	return colorFound


# Function to run the object's detection on a given image
	# INPUT : the name (path) of an image, example ./Images/rouge.jpg
def run(imageName):
	cd = ColorDetector()

	# DETECTION FROM AN IMAGE WITH MEANSHIFT
	#blobs = cd.runCDImageMS(imageName)	
	
	# DETECTION FROM EN IMAGE WITH SUPERPIXELS
	#blobs = cd.runCDImageSLIC(imageName)
	
	# DETECTION FROM POINT CLOUD WITH MEANSHIFT

	x = np.array([45.2,65.3, 78, 15,0, 26.4,78.9,12.6,45.1,62,51,78.1])
	y = np.array([17.3,56.8, 9.1,25,   48.6,12.7,3,9,89.4,19.7,65,14])
	z = np.array([21,  46.7, 84, 24.3, 76.5,20,41.2,73.6,81,67.6,14.2,56.1])
	
	r = np.array([225,255,0,0,0,255,0,0,255,0,0,0])
	g = np.array([10,0,0,255,255,0,0,0,255,0,255,0])
	b = np.array([10,0,255,0,0,0,255,255,0,0,0,255])
	
	
	labels = np.array([0,0,2,1,0,0,2,2,2,2,1,1])
	
	d = np.array([x,y,z,r,g,b,labels]).transpose()

	ColorSeg = True
	blobs = cd.runCDData(d, ColorSeg)

def visualizeResultsBar(blobs, size, xBarycentre, yBarycentre):
	nbRow = size[0]
	nbCol = size[1]
	nbCha = size[2]
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
	
	imageResult = imageResult.reshape(nbRow,nbCol,nbCha)
	imageResult[xBarycentre, yBarycentre, 0] = 255
	imageResult[xBarycentre, yBarycentre, 1] = 255
	imageResult[xBarycentre, yBarycentre, 2] = 255
	imageResult = np.uint8(imageResult)
	imageResult = Image.fromarray(imageResult)
	#imageResult.show()	
	imageResult.save('/home/n7/resultColor.png')


if __name__ == '__main__':
	tic = time.clock()
	run(sys.argv[1])
	toc = time.clock()
	print toc - tic
