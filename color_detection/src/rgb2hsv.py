import numpy as np

#Function that takes a RGB image and returns the corresponding HSV image
	# INPUT : (nbPixels,3) data, the image to convert
	#	      size, the size of the image
	# OUTPUT : (nbPixels,3) imageHSV, the HSV image as an array
	# see http://www.rapidtables.com/convert/color/rgb-to-hsv.htm for more explanations
def rgb2hsv(data, size):

	nbRow, nbCol, nbCha = size
	nbPix = nbRow * nbCol
	
	#Computation of internal parameters
	normalizedImage = data/255
	cMax = np.max(normalizedImage,axis=1)
	cMin = np.min(normalizedImage,axis=1)
	delta = cMax - cMin

	# Hue 
	H = np.zeros(nbPix)
	rPrime = normalizedImage[:,0]
	gPrime = normalizedImage[:,1]
	bPrime = normalizedImage[:,2]

	indices = (delta != 0) * (cMax == rPrime)
	H[indices] = 60*((np.divide(gPrime[indices] - bPrime[indices], delta[indices]))%6)
	
	indices = (delta != 0) * (cMax == gPrime)
	H[indices] = 60*((np.divide(bPrime[indices] - rPrime[indices], delta[indices]))+2)

	indices = (delta != 0) * (cMax == bPrime)
	H[indices] = 60*((np.divide(rPrime[indices] - gPrime[indices], delta[indices]))+4)
	
	# Saturation
	S = np.zeros(nbPix)

	S[(cMax != 0)] = np.divide(delta[(cMax != 0)], cMax[(cMax != 0)])

	# Value
	V = cMax
	
	imageHSV = np.array([H,S,V]).transpose()
	
	return imageHSV

#Function that takes a RGB pixel and returns the corresponding HSV pixel
	# INPUT : (1,3) pixels, the pixel to convert
	# OUTPUT : (1,3) hsvPixel, the HSV pixel
	# see http://www.rapidtables.com/convert/color/rgb-to-hsv.htm for more explanations	
def rgb2hsvPix(pixel):
	
	#Computation of internal parameters
	normalizedPixel = pixel/255
	cMax = np.max(normalizedPixel)
	cMin = np.min(normalizedPixel)
	delta = cMax - cMin

	# Hue 
	rPrime = normalizedPixel[0]
	gPrime = normalizedPixel[1]
	bPrime = normalizedPixel[2]
	
	H = 0
	if delta != 0:
		if cMax == rPrime:
			H = 60*(((gPrime - bPrime) / delta) % 6)
		elif cMax == gPrime:
			H = 60*(((bPrime - rPrime) / delta) + 2)
		else:
			H = 60*(((rPrime - gPrime) / delta) + 4)

	# Saturation
	S = 0
	if cMax != 0:
		S = delta/cMax

	# Value
	V = cMax
	
	hsvPixel = [H, S, V]
	
	return hsvPixel
