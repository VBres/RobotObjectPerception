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
#include <messages/PointCloudObjectSeq.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


float cloudRemaining = 0.20; // Percentage of points remaining after the suppression of the largest components
float clusterDistance = 0.01; // Minimal distance between two different clusters, in meters
int minSize = 15; // Clusters with less than minSize points are excluded
int maxSize = 1000; // CLusters with more then maxSize points are excluded
float detectionRange = 2.0; // Maximum distance of detection, in meters







ros::Publisher pub;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) // callback doing the actual computing
{
	// Container for original data
	pcl::PCLPointCloud2* cloud_pc2 = new pcl::PCLPointCloud2; 


	// Convert to PCL data type
	PointCloudT::Ptr cloud (new PointCloudT), cloud_f (new PointCloudT);
	pcl_conversions::toPCL(*cloud_msg, *cloud_pc2);
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
	while (cloud_filtered->points.size() > cloudRemaining*nr_points)
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


	int j = 0;  // Only useful if you want to save the clusters
	messages::PointCloudObjectSeq output;



	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{	
		bool far = false; // True if the object is too far away
		//Loop on clusters
		PointCloudT::Ptr cloud_cluster (new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
			float dist = sqrt((cloud_filtered->points[*pit].x)*(cloud_filtered->points[*pit].x) + (cloud_filtered->points[*pit].y)*(cloud_filtered->points[*pit].y) + (cloud_filtered->points[*pit].z)*(cloud_filtered->points[*pit].z));
			if (dist > detectionRange ) {		
				far = true;
			}
		}

		if (not far) {
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;




			size_t size2(cloud_cluster->points.size());

			// IF YOU WANT TO SAVE THE DIFFERENT CLUSTERS IN ORDER TO SEE THEM WITH PCL_VIEWER, UNCOMMENT THE FOLLOWING LINES AND REPLACE "YOURPATH" WITH WHERE YOU WANT TO SAVE THEM
			/*
			std::stringstream ss;
			ss << "YOURPATH/cloud_cluster_" << j << ".pcd";
			writer.write<PointT> (ss.str (), *cloud_cluster, false);
       			*/

			for (size_t i = 0; i<size2; i++) {
				//Fill-in the output list
				output.Xlist.push_back(cloud_cluster->points[i].x);
				output.Ylist.push_back(cloud_cluster->points[i].y);
				output.Zlist.push_back(cloud_cluster->points[i].z);
				output.Rlist.push_back(cloud_cluster->points[i].r);
				output.Glist.push_back(cloud_cluster->points[i].g);
				output.Blist.push_back(cloud_cluster->points[i].b);
				output.Clusters.push_back(j);
			}

			j++; // Only useful if you want to save the clusters

		}

	}

	std::cout<<"found "<<j<< " estimated objects\n";
	// Publish the data
	pub.publish (output);
}

	int
main (int argc, char** argv)
{
	// Initialize ROS
	std::cout<<"pclSequential running \n";
	ros::init (argc, argv, "pcl_sequential");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("dispatcher_point_cloud", 1, cloud_cb);

	// Create a ROS publisher for the output list
	pub = nh.advertise<messages::PointCloudObjectSeq> ("point_cloud_detection", 1);

	// Spin
	ros::spin ();
}

