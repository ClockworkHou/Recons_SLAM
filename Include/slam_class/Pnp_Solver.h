#ifndef Pnp_Solver_H
#define Pnp_Solver_H

#include"Map.h"
#include"../include_libs.h"
//#include"MapPoint.h"

namespace slam_class
{
	
class Pnp_Solver
{
public:
	Map* Global_Map;
	Map* Local_Map;
	unsigned long cur_frame_id;
	unsigned long ref_frame_id;
	vector<DMatch> match;
	vector<DMatch> matches;
	
	//pnp
	vector<KeyPoint> keypoints;
	vector<KeyPoint> keypoints_1;
	vector<KeyPoint> keypoints_2;
	vector< Point3f > points_3d;
	vector< Point2f > points_2d;
	//triangulation
	vector<Point3d> points;
	vector<Point2d> pts_1;
	vector<Point2d> pts_2;
	vector<Mat>  descriptors_buf;
	vector<unsigned long> delete_list;
	
	 
	
	void  find_feature_matches ( 
			    const Mat& img,
			    Mat& map_descriptors,
			    Mat& descriptors
       				);
	Point2d pixel2cam ( const Point2d & p, const Mat& K );
	
	
	 void bundleAdjustment (
		const Mat& K,
		Mat& R, Mat& t );
	
	void triangulate(
		const Mat & img_1,
		const Mat & img_2,
		const Mat & R,
		const Mat & t,
		const SE3 & Tcw1,
		const SE3 & Tcw2
	);
	
	void Solve_Pnp (unordered_map<unsigned long, Frame*>::iterator it1, 
						unordered_map<unsigned long, Frame*>::iterator it2
	);
	
	void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t );
	void run();
	void update_local_map();
	
	
};

}

#endif