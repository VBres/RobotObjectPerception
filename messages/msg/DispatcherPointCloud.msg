# This is the point cloud data given by the dispatcher to the point cloud detection in parallel 

# A header where the stamp is modified so the depth data and the rgb data match
std_msgs/Header header

# The point clouds from the depth camera
sensor_msgs/PointCloud2 pointCloud
