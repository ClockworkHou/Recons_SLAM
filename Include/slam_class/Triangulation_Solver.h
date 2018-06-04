//calcualte initial map with 8-points/5-points algorithm

#ifndef Triangulation_Solver_H
#define Triangulation_Solver_H

#include"../include_libs.h"
#include"Map.h"
//#include"MapPoint.h"

namespace slam_class
{
	
class Triangulation_Solver
{
public:
	Map* Global_Map;
	Map* Local_Map;
	
	Point2d pixel2cam ( const Point2d & p, const Mat& K );
	
	void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches ,
			    Mat& descriptors_1,
			    Mat& descriptors_2
  				);
	void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t );
	void triangulate ( 
		const vector< KeyPoint >& keypoint_1, 
		const vector< KeyPoint >& keypoint_2, 
		const std::vector< DMatch >& matches,
		const Mat& R, const Mat& t, 
		vector< Point3d >& points,
		Mat&  descriptors_1,
		vector<Mat> & descriptors
 			);
	void initialize();
	
};

}

#endif