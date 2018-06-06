
#include"../Include/slam_class/Map_Viewer.h"
#define scale_adjustment  1

namespace slam_class 
{
void Map_Viewer::run(char* filename)
{
	
	pcl::PointCloud<pcl::PointXYZ> cloud; 
	
	// Fill in the cloud data  
	
	
	int num = Global_Map->map_points.size();
	cloud.width    = num;
	cloud.height   = 1;  
	cloud.is_dense = false; 
	cloud.points.resize (num);  
	
	for (size_t i = 0; i < num; ++i)  
	{  
		cloud.points[i].x = Global_Map->map_points.at(i)->pos(0, 0);
		cloud.points[i].y = Global_Map->map_points.at(i)->pos(1, 0);
		cloud.points[i].z = Global_Map->map_points.at(i)->pos(2, 0);
		//cout <<  Global_Map->map_points.at(i)->pos(0, 0) << endl;
		//cout <<  Global_Map->map_points.at(i)->pos(1, 0)<< endl;
		//cout <<  Global_Map->map_points.at(i)->pos(2, 0)<< endl;
	}  
	
	pcl::io::savePCDFileASCII (filename, cloud);  
	//std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;  
	
	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");//直接创造一个显示窗口 
	
	//viewer.showCloud(cloud.makeShared(), "recons"); //显示点云  
	
	//while (!viewer.wasStopped())  {};
//  {  
//  }  
	//for (size_t i = 0; i < cloud.points.size (); ++i)  
	//  std::cerr << " " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;  
	
	//getchar();  
	return;  
}

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
}