#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

//void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
//{
//	viewer.setBackgroundColor (5, 5, 5);
//}

//void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
//{
//	static unsigned count = 0;
//	std::stringstream ss;
//	ss << "Once per viewer loop: " << count++;
//	viewer.removeShape ("text", 0);
//	viewer.addText (ss.str(), 200, 300, "text", 0);
//}

int main (int argc, char *argv[]) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::visualization::PCLVisualizer viewer ("Viewer");
	//viewer.setBackgroundColor (10, 10, 10);

	int i = 0;
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"a");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"b");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"c");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"d");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"e");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"f");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"g");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"h");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"i");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"j");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"k");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"l");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"m");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"n");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"o");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"p");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"q");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"r");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"s");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"t");
	i++;
	}	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"u");
	i++;
	}	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"v");
	i++;
	}	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"w");
	i++;
	}	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"x");
	i++;
	}	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"y");
	i++;
	}	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"z");
	i++;
	}	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"1");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"2");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"3");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"4");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"5");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"6");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"7");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"8");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"9");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"10");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"11");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"12");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"13");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"14");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"15");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"16");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"17");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"18");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"19");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"20");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"21");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"22");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"23");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"24");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"25");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"26");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"27");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"28");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"29");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"30");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"31");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"32");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"33");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"34");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"35");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"36");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"37");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"38");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"39");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"40");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"41");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"42");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"43");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"44");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"45");
	i++;
	}
	if (i<argc) {
	pcl::io::loadPCDFile (argv[i], *cloud);
	viewer.addPointCloud (cloud,"46");
	i++;
	}


	
	//viewer.runOnVisualizationThreadOnce (viewerOneOff);
        //viewer.runOnVisualizationThread (viewerPsycho);

	while (!viewer.wasStopped ())
	{
	viewer.spinOnce ();
	}



	return 0;

}
