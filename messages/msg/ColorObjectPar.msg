# Message given by the color detection to the merger in the parallel version
# In each list, the same index corresponds to same detected object

# List of the colors of the detected objects
# 0 is yellow, 1 is red, 2 is green, 3 is blue, 4 is purple, 5 is orange
int8[] colorList 

# List of the coordinates of the barycenter of the detected objects in pixel in the rgb image
float32[] xCentList 
float32[] yCentList 

# List of number of pixels of the detected objects int the rgb image
int32[] nbPixList

std_msgs/Header header
