import numpy as np

# Function that takes an index in a vectorized array and returns the corresponding row and column in the reshaped array
	# INPUT : size, the size of the reshaped array (nbRow, nbCol)
	#		  ind, the index in the vectorized array
	# OUTPUT : 	row and col, the corresponding row and column
def ind2sub(size, ind):
    row = (ind.astype('int') / size[1])
    col = (ind.astype('int') % size[1]) 
    return (row, col)
 
# Function that takes a row and a column in a nbRow x nbCol array  and returns the corresponding index in the vectorized
	# INPUT : size, the size of the reshaped array (nbRow, nbCol)
	#		  row, the row
	#		  col, the column
	# OUTPUT : ind,	the corresponding index  
def sub2ind(size, row, col):
	ind = row*size[1] + col
	return ind
	
# Function that returns the index in the list of the blobs that are of the given color
	# INPUT : blobs, a list of blobs with their color, ex: [[blue,[index of pixels]],[red,[index of pixels]]]
	#	      color, the color we look for
	# OUTPUT : index, a list of the corresponding index of the blobs in the given list
def getColoredBlobs(blobs, color):
	index = []
	for i in range(len(blobs)):
		if color in blobs[i] : 
			index.append(i)
	return index

# Function that defines if two blobs are neighbours
	# INPUT : blob1, it contains xmin xmax ymin ymax of the first blob
	#		  blob2, it contains xmin xmax ymin ymax for the second blob
	# OUTPUT : result, true if the two blobs are neighbours
def areNeighbours2D(blob1, blob2):
	xmin1 = blob1[0]
	xmax1 = blob1[1]
	ymin1 = blob1[2]
	ymax1 = blob1[3]
	
	xmin2 = blob2[0]
	xmax2 = blob2[1]
	ymin2 = blob2[2]
	ymax2 = blob2[3]
	
	cond1 = ((xmin2 <= xmin1 <= xmax2) or (xmin2 <= xmax1 <= xmax2)) and ((ymin2 <= ymin1 <= ymax2) or (ymin2 <= ymax1 <= ymax2))
	cond2 = ((xmin1 <= xmin2 <= xmax1) or (xmin1 <= xmax2 <= xmax1)) and ((ymin1 <= ymin2 <= ymax1) or (ymin1 <= ymax2 <= ymax1))
	result = cond1 or cond2
	
	return result
