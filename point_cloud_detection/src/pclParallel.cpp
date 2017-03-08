#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <messages/PointCloudObjectPar.h>
#include <messages/DispatcherPointCloud.h>
#include <math.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/region_growing_rgb.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



float cloudRemaining = 0.20; // Percentage of points remaining after the suppression of the largest components
float clusterDistance = 0.01; // Minimal distance between two different clusters, in meters
int minSize = 15; // Clusters with less than minSize points are excluded
int maxSize = 1000; // CLusters with more then maxSize points are excluded
float detectionRange = 2.0; // Maximum distance of detection, in meters


ros::Publisher pub;

float* wrapping (PointCloudT::Ptr cloud_in) {
	//Function that computes the barycenter and the bounding box of the input cloud

	size_t size(cloud_in->points.size());
	float min_x(cloud_in->points[0].x);
	float min_y(cloud_in->points[0].y);
	float min_z(cloud_in->points[0].z);
	float max_x(cloud_in->points[0].x);
	float max_y(cloud_in->points[0].y);
	float max_z(cloud_in->points[0].z);
	float sum_x(cloud_in->points[0].x);
	float sum_y(cloud_in->points[0].y);
	float sum_z(cloud_in->points[0].z);
	float sum_r;
	sum_r = (float)(cloud_in->points[0].r);
	float sum_g;
	sum_g = (float)(cloud_in->points[0].g);
	float sum_b;
	sum_b = (float)(cloud_in->points[0].b);
	for (size_t i = 1; i < size; ++i)
	{
		if (cloud_in->points[i].x < min_x)
		{
			min_x = cloud_in->points[i].x;
		}
		if (cloud_in->points[i].y < min_y)
		{
			min_y = cloud_in->points[i].y;
		}
		if (cloud_in->points[i].z < min_z)
		{
			min_z = cloud_in->points[i].z;
		}
		if (cloud_in->points[i].x > max_x)
		{
			max_x = cloud_in->points[i].x;
		}
		if (cloud_in->points[i].y > max_y)
		{
			max_y = cloud_in->points[i].y;
		}
		if (cloud_in->points[i].z > max_z)
		{
			max_z = cloud_in->points[i].z;
		}
		sum_x = sum_x+cloud_in->points[i].x;
		sum_y = sum_y+cloud_in->points[i].y;
		sum_z = sum_z+cloud_in->points[i].z;
		sum_r = sum_r+(float)(cloud_in->points[i].r);
		sum_g = sum_g+(float)(cloud_in->points[i].g);
		sum_b = sum_b+(float)(cloud_in->points[i].b);
	} 
	float moy_x(sum_x/size);
	float moy_y(sum_y/size);
	float moy_z(sum_z/size);
	float moy_r(sum_r/size);
	float moy_g(sum_g/size);
	float moy_b(sum_b/size);

	float* res = new float[12];
	res[0] = moy_x;
	res[1] = moy_y;
	res[2] = moy_z;
	res[3] = min_x;
	res[4] = max_x;
	res[5] = min_y;
	res[6] = max_y;
	res[7] = min_z;
	res[8] = max_z;
	res[9] = moy_r;
	res[10] = moy_g;
	res[11] = moy_b;


	return (res);

}

void cloud_cb (messages::DispatcherPointCloud cloud_msg)
{  //callback doing the actual computation

	sensor_msgs::PointCloud2 cloud_in = cloud_msg.pointCloud;
	std_msgs::Header header = cloud_msg.header;

	// Container for original data
	pcl::PCLPointCloud2* cloud_pc2 = new pcl::PCLPointCloud2; 


	// Convert to PCL data type
	PointCloudT::Ptr cloud (new PointCloudT), cloud_f (new PointCloudT);
	pcl_conversions::toPCL(cloud_in, *cloud_pc2);
	pcl::fromPCLPointCloud2(*cloud_pc2, *cloud);


	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<PointT> vg;
	PointCloudT::Ptr cloud_filtered (new PointCloudT);
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.007f, 0.007f, 0.007f);
	vg.filter (*cloud_filtered);

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	PointCloudT::Ptr cloud_plane (new PointCloudT ());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	int i=0, nr_points = (int) cloud_filtered->points.size ();
	while (cloud_filtered->points.size () > cloudRemaining * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Get the points associated with the planar surface
		extract.filter (*cloud_plane);

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud_filtered = *cloud_f;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (clusterDistance); 
	ec.setMinClusterSize (minSize);
	ec.setMaxClusterSize (maxSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);

	size_t size(cloud_filtered->points.size() );

	int j = 0;
	messages::PointCloudObjectPar output;


	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{	
		bool far = false; // True if the object is too far away

		PointCloudT::Ptr cloud_cluster (new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
			float dist = sqrt((cloud_filtered->points[*pit].x)*(cloud_filtered->points[*pit].x) + (cloud_filtered->points[*pit].y)*(cloud_filtered->points[*pit].y) + (cloud_filtered->points[*pit].z)*(cloud_filtered->points[*pit].z));
			if (dist > detectionRange) {
				far = true;
			}
		}
		if (not far) {
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;


			pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
			pcl::RegionGrowingRGB<PointT> reg;
			reg.setInputCloud (cloud_cluster);
			reg.setSearchMethod (tree);
			reg.setDistanceThreshold (2);
			reg.setPointColorThreshold (20);
			reg.setRegionColorThreshold (10);
			reg.setMinClusterSize (minSize);

			std::vector <pcl::PointIndices> color_indices;
			reg.extract (color_indices);

			for(std::vector<pcl::PointIndices>::const_iterator it_col = color_indices.begin (); it_col != color_indices.end (); ++it_col) {	
				PointCloudT::Ptr color_cluster (new PointCloudT);
				for (std::vector<int>::const_iterator pit = it_col->indices.begin (); pit != it_col->indices.end (); ++pit) {
					color_cluster->points.push_back (cloud_cluster->points[*pit]);
				}
				color_cluster->width = color_cluster->points.size ();
				color_cluster->height = 1;
				color_cluster->is_dense = true;


				// IF YOU WANT TO SAVE THE DIFFERENT CLUSTERS IN ORDER TO SEE THEM WITH PCL_VIEWER, UNCOMMENT THE FOLLOWING LINES AND REPLACE "YOURPATH" WITH WHERE YOU WANT TO SAVE THEM		
				/*
				std::stringstream ss;
				ss << "/YOURPATH/cloud_cluster_" << j << ".pcd";
				writer.write<PointT> (ss.str (), *color_cluster, false);
				*/

				// Find barycenters and bounding box
				float* res = new float[12];
				res = wrapping(color_cluster);
				output.Xlist.push_back(res[0]);
				output.Ylist.push_back(res[1]);
				output.Zlist.push_back(res[2]);
				output.MinX.push_back(res[3]);
				output.MaxX.push_back(res[4]);
				output.MinY.push_back(res[5]);
				output.MaxY.push_back(res[6]);
				output.MinZ.push_back(res[7]);
				output.MaxZ.push_back(res[8]);
				output.RList.push_back(res[9]);
				output.GList.push_back(res[10]);
				output.BList.push_back(res[11]);


				j++;
			}
		}
	}
	std::cout<<"found "<<j<< " estimated objects\n";
	output.header = header;
	// Publish the data
	pub.publish (output);
}

	int
main (int argc, char** argv)
{
	// Initialize ROS
	std::cout<<"pclParallel running \n";
	ros::init (argc, argv, "pcl_parallel");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("dispatcher_point_cloud", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<messages::PointCloudObjectPar> ("point_cloud_detection", 1);

	// Spin
	ros::spin ();
}
