from rgb2hsv import *
	
# Function that takes a color and determines in which range of defined colors it belongs using HSV space
	# INPUT : color, the given color RGB
	#		  activatedColors, an array like [(yellow,bool),(red,bool),(green,bool),(blue,bool),(orange,bool),(purple,bool)
	# OUTPUT : colorFound, a number corresponding to the color found
	
def findColorRangeHSV(color, activatedColors):
    
	# Tranform the rgb color into a hsv color
	HSVcolor = rgb2hsvPix(color)
	
	# Get the hue and saturation
	h = HSVcolor[0]
	s = HSVcolor[1]
	
	
	# This threshold helps to distinguish colors and white/black
	saturationThreshold = 0.35
	
	colorFound = -1
	if s > saturationThreshold:
		if (43 <= h < 76) and activatedColors[0] :
			# Yellow
			colorFound = 0
		elif ((320 <= h <= 360) or (0 <= h < 14))  and activatedColors[1] :
			# Red
			colorFound = 1
		elif (76 <= h < 200) and activatedColors[2] :
			# Green
			colorFound = 2
		elif (200 <= h < 260) and activatedColors[3] :
			# Blue
			colorFound = 3
		elif (14 <= h < 43) and activatedColors[4] :
			# Orange
			colorFound = 4
		elif (260 <= h < 320) and activatedColors[5] :
			# Purple
			colorFound = 5
			
	return colorFound
	

