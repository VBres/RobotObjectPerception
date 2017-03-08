#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <math.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

string db_path="/home/n7/Database/cylinder_";
int nb_cyl = 20; // nb_cyl != 0;

int main (int argc, char** argv) {

	pcl::PCDWriter writer;
	float radius;
	for (int i(0); i < nb_cyl; i+=1)
	{
		radius = 0.05*(float)(1 + i ) / (float)(nb_cyl);
		//aux = 10.0;
		
		// Create the sources
		PointCloudT::Ptr cylinder (new PointCloudT);
		for (float angle(0.0); angle <= 360.0; angle += 20.0)
		{	
			for (float r(0.0); r<radius; r+= radius/5)
			{
				PointT basic_point;
				basic_point.x = r*cosf (2.0*M_PI*angle/360.0);
				basic_point.y = 0.0*radius;
				basic_point.z = r*sinf (2.0*M_PI*angle/360.0);
				cylinder->points.push_back(basic_point);

				/*basic_point.x = r*cosf (2.0*M_PI*angle/360.0);
				basic_point.y = 0.5*radius;
				basic_point.z = r*sinf (2.0*M_PI*angle/360.0);
				cylinder->points.push_back(basic_point);

			}
			for (float y(-0.5*radius); y <= (0.5*radius); y += (0.1*radius))
			{
				PointT basic_point;
				basic_point.x = radius*cosf (2.0*M_PI*angle/360.0);
				basic_point.y = y;
				basic_point.z = radius*sinf (2.0*M_PI*angle/360.0);
				cylinder->points.push_back(basic_point);

			*/}
		}
		cylinder->width = (int) cylinder->points.size ();
		cylinder->height = 1;

		std::stringstream ss;
		ss<< db_path << i << ".pcd";
		writer.write<PointT> (ss.str (), *cylinder, false); //*
	}

	return(0);
}
