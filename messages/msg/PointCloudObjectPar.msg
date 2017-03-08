# Result of the point cloud detection in parallel given to the merger
# In each list, the same index corresponds to same detected object

# Lists containing the coordinates of the objects' barycenters in the 3D point cloud reference
float32[] Xlist
float32[] Ylist
float32[] Zlist

# Lists containing the objects' bounding boxes in the 3D point cloud reference
float32[] MinX
float32[] MaxX
float32[] MinY
float32[] MaxY
float32[] MinZ
float32[] MaxZ

# Lists containg the objects' RBG chanels
int32[] RList
int32[] GList
int32[] BList

# The stamp has been modified to synchronize the rgb image and the depth image
std_msgs/Header header
