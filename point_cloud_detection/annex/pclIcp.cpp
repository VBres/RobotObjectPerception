/* Execute the icp algorithm on the cloud given in input */


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
#include <pcl_icp2/OutputType.h>
#include <math.h>
#include <pcl/registration/icp.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

string db_path="/home/n7/Database/cylinder_";
string result_path="/home/n7/icp_on_cylinder_";
int nb_cyl = 20;

float* wrapping (PointCloudT::Ptr cloud_in) {
	//Function that computes the barycenter and the bounding of the input cloud

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
	} 
	float moy_x(sum_x/size);
	float moy_y(sum_y/size);
	float moy_z(sum_z/size);

	float* res = new float[9];
	res[0] = moy_x;
	res[1] = moy_y;
	res[2] = moy_z;
	res[3] = min_x;
	res[4] = max_x;
	res[5] = min_y;
	res[6] = max_y;
	res[7] = min_z;
	res[8] = max_z;


	return (res);

}

void translate (PointCloudT::Ptr cloud, float offsetX, float offsetY, float offsetZ)
{
	size_t size(cloud->points.size());
	for(size_t i(0); i<size; i++) {
		cloud->points[i].x+= offsetX;
		cloud->points[i].y+= offsetY;
		cloud->points[i].z+= offsetZ;
	}
 }

void cloud_cb (PointCloudT::Ptr cloud)
{
	float* wrapped = new float[9];
	wrapped = wrapping(cloud);

	//Perform the icp algorithm

	float score = 10000000000000000000.0;
	int j=0;

	for (int i(0); i  < nb_cyl ; i+=1)
	{
		pcl::PCDReader reader;
		PointCloudT::Ptr cylinder (new PointCloudT);

		std::stringstream ss;
		ss << db_path << i << ".pcd";

		reader.read (ss.str (), *cylinder);
		
		translate(cylinder, (wrapped[3]+wrapped[4])/2, (wrapped[5]+wrapped[6])/2, (wrapped[7]+wrapped[8])/2);

		PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
		pcl::IterativeClosestPoint<PointT, PointT> icp;
		*cloud_icp = *cylinder;
		icp.setInputSource (cylinder);
		icp.setInputTarget (cloud);

		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (1);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations (100);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon (1e-8);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon (1);
		// Perform the alignment
		icp.align (*cloud_icp);
		// Obtain the transformation that aligned cloud_source to cloud_source_registered
		Eigen::Matrix4f transformation = icp.getFinalTransformation ();
		// if icp.getFitnessScore ()
		std::cout<< "\nscore of " << i << " : " << icp.getFitnessScore();
		if (score > icp.getFitnessScore()) {
			score = icp.getFitnessScore();
			j=i;
		}
		else {}
		pcl::PCDWriter writer;
		std::stringstream ss2;
		ss2 << result_path << i << ".pcd";
		writer.write<PointT> (ss2.str (), *cloud_icp, false);

	}

	std::cout<<"best cylinder is no "<<j<<"\n";
	




}

	int
main (int argc, char** argv)
{
	PointCloudT::Ptr cloud_in (new PointCloudT);
	pcl::PCDReader reader;
	reader.read (argv[1], *cloud_in);
	cloud_cb(cloud_in);
	return(0);
}

