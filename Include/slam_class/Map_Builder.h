#ifndef Map_Builder_H
#define Map_Builder_H

#include"../include_libs.h"
#include"Map.h"
//#include"MapPoint.h"

namespace slam_class
{
	
class Map_Builder
{
public:
	Map* Global_Map;
	Map* Local_Map;
	unsigned long cur_frame_id;
	unsigned long ref_frame_id;
	vector<DMatch> match;
	vector<DMatch> matches;
	
	//
	vector<KeyPoint> keypoints;
	vector<KeyPoint> keypoints_1;
	vector<KeyPoint> keypoints_2;
	vector< Point3d > points_3d;
	vector< Point2d> points_2d;
	vector<Point2d> points1;
	vector<Point2d> points2;
	//triangulation
	vector<Point3d> points;
	vector<Point2d> pts_1;
	vector<Point2d> pts_2;
	vector<Mat>  descriptors_buf;
	vector<unsigned long> delete_list;
	
	 
	Point2d pixel2cam ( const Point2d & p, const Mat& K );
	
	/*
	 void bundleAdjustment (
		const Mat& K,
		Mat& R, Mat& t );
	*/
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
		const Mat& Tcw1, const Mat& Tcw2, 
		const Mat& R, const Mat& t,
		vector< Point3d >& points,
		Mat&  descriptors_1,
		vector<Mat> & descriptors
 			);
	
	void Solve_2d2d (unordered_map<unsigned long, Frame*>::iterator it1, 
						unordered_map<unsigned long, Frame*>::iterator it2
	);
	
	void run();
	void update_local_map();
	
	
};

}

#endif