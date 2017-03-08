# Message containing the result of the whole detection (color + point cloud)
# It contains the information about one object detected

# These are the coordinates of the barycentre in the 3D point cloud reference
float32 xCentroid
float32 yCentroid
float32 zCentroid

# These are the information about the bounding box of the object (in the 3D point cloud reference)
float32 xMin
float32 xMax
float32 yMin
float32 yMax
float32 zMin
float32 zMax

# This is the color of the object
# 0 is yellow, 1 is red, 2 is green, 3 is blue, 4 is purple, 5 is orange
int8 color
