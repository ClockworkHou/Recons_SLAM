#include"../Include/slam_class/Map_Builder.h"
#include"../Include/slam_class/config_hc.h"

#include <iostream>


namespace slam_class 
{
void Map_Builder::Solve_2d2d (unordered_map<unsigned long, Frame*>::iterator it1, 
						unordered_map<unsigned long, Frame*>::iterator it2
)
{
	
   Mat img_1 = it1->second->rgbImg;
   Mat img_2 = it2->second->rgbImg;
   
    keypoints_1.clear();
    keypoints_2.clear();;
    matches.clear();
    Mat descriptors_1,descriptors_2;
    
    find_feature_matches ( img_1, img_2, 
			   keypoints_1, keypoints_2,
			   matches, 
			   descriptors_1,descriptors_2 ); 
    
    //估计两张图像间运动
  
    Mat R,t;
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );
   
    //计算当前帧的位姿
    SE3 se3_Tcw1 = it1->second->T_c_w;
    MatrixXd tmp_m = se3_Tcw1.matrix();
    
    Mat Tcw1 = (Mat_<double> (4,4) <<
         tmp_m(0,0), tmp_m(0,1), tmp_m(0,2),tmp_m(0,3),
         tmp_m(1,0), tmp_m(1,1), tmp_m(1,2),tmp_m(1,3),
         tmp_m(2,0), tmp_m(2,1), tmp_m(2,2),tmp_m(2,3),
		0,0,0,1
    );
    Mat Tcw2 = (Mat_<double> (4,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0),
		0,0,0,1
    );
    
    Tcw2 = Tcw2 * Tcw1;
    
    //将位姿写入cur_frame
    Matrix3d rotation;  
//PB
    rotation << Tcw2.at<double>(0,0), Tcw2.at<double>(0,1),  Tcw2.at<double>(0,2), 
			Tcw2.at<double>(1,0), Tcw2.at<double>(1,1),  Tcw2.at<double>(1,2),
			Tcw2.at<double>(2,0), Tcw2.at<double>(2,1),  Tcw2.at<double>(2,2);
   Vector3d translation;
   translation << Tcw2.at<double>(0,3), Tcw2.at<double>(1,3), Tcw2.at<double>(2,3);
			
    it2->second->T_c_w = SE3(rotation, translation);
    
    //验证E=t^R*scale
    /*Mat t_x = ( Mat_<double> ( 3,3 ) <<
                0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
                t.at<double> ( 2,0 ),      0,                      -t.at<double> ( 0,0 ),
                -t.at<double> ( 1.0 ),     t.at<double> ( 0,0 ),      0 );
	*/
    //cout<<"t^R="<<endl<<t_x*R<<endl;

    //-- 验证对极约束
    //Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    /*Mat K = ( Mat_<double> ( 3,3 ) << 
		camera_Intrinsics_FX, 0, camera_Intrinsics_CX, 
	        0, camera_Intrinsics_FY, camera_Intrinsics_CY, 
	        0, 0, 1 );
    for ( DMatch m: matches )
    {
        Point2d pt1 = pixel2cam ( keypoints_1[ m.queryIdx ].pt, K );
        Mat y1 = ( Mat_<double> ( 3,1 ) << pt1.x, pt1.y, 1 );
        Point2d pt2 = pixel2cam ( keypoints_2[ m.trainIdx ].pt, K );
        Mat y2 = ( Mat_<double> ( 3,1 ) << pt2.x, pt2.y, 1 );
        Mat d = y2.t() * t_x * R * y1;
        //cout << "epipolar constraint = " << d << endl;
    }*/
    
    points.clear();
    descriptors_buf.clear();
    
    //三角测量求解做标点位置
    triangulate ( keypoints_1, keypoints_2, 
		matches, 
		  Tcw1,  Tcw2,  
		  R, t,
		  points,
		descriptors_1,
		descriptors_buf
		);
    
    //保存地标点
     SE3 se3_Tcw2 = it1->second->T_c_w;
    MatrixXd tmp_t = se3_Tcw2.translation();
    double cx = tmp_t(0,0);
    double cy = tmp_t(1,0);
    double cz = tmp_t(2,0);
    
    for( int i = 0; i < points.size(); i++)
    {
	     if( abs(points.at(i).x) > BADPOINT_THRESHOLD ||
		    abs(points.at(i).y) > BADPOINT_THRESHOLD ||
		    abs(points.at(i).z) >BADPOINT_THRESHOLD)
		    continue;
	     double d = ((points.at(i).x-cx) *(points.at(i).x-cx) 
		     +  (points.at(i).y-cy) *(points.at(i).y-cy) 
		     +  (points.at(i).z-cz) *(points.at(i).z-cz));
	     if  ( d > BADPOINT_THRESHOLD || d < 0.05) continue;
		     
		    
	    MapPoint * tmp = new MapPoint();
	     
	    tmp->id = Global_Map->map_points.size();
	    tmp->pos(0,0) = points.at(i).x;
	    tmp->pos(1,0) = points.at(i).y;
	    tmp->pos(2,0) = points.at(i).z;
	    tmp->is_good = true;
	    
	    unordered_map<unsigned long, Frame*>::iterator it;
	   
	    tmp->observed_frames.push_back(it1->second);
	    
	    tmp->observed_frames.push_back(it2->second);
	    
	    tmp->descriptor = descriptors_buf.at(i);
	    tmp->matched_times = 1;
	    tmp->visible_times = 1; 
	    Global_Map->insertMapPoint(tmp);
	    Local_Map->insertMapPoint(tmp);
	    
    }
    //局部地图 
   
     Local_Map->insertKeyFrame(it2->second);
     
     
     return ;
}
   
		
	
void Map_Builder::update_local_map()
{
	bool flag;
	delete_list.clear();
	
	//找到需要删除的点
	unordered_map<unsigned long, MapPoint* >::iterator tmp_it;
    
	for( tmp_it = Local_Map->map_points.begin(); tmp_it != Local_Map->map_points.end(); tmp_it++)
	//for( int i = 0; i < Local_Map->map_points.size(); i++)
	{
		flag = false;
		list< Frame*>::iterator iter;
		//for( iter = Local_Map->map_points.at(i)->observed_frames.begin(); iter != Local_Map->map_points.at(i)->observed_frames.end(); iter++) 
		//for(int j = 0; j < Local_Map->map_points.at(i)->observed_frames.size();j++)
		for( iter = tmp_it->second->observed_frames.begin(); iter != tmp_it->second->observed_frames.end(); iter++) 
		{
			unsigned long tmpid =  (*iter)->id;
			if(
				((tmpid >= (cur_frame_id - LOCALMAP_FRAMES))|| (cur_frame_id <= LOCALMAP_FRAMES)) 
				&& (tmpid <= cur_frame_id)
			)
			{
				flag = true;
				break;
			}
		}
		if(!flag)
		{
			delete_list.push_back(tmp_it->second->id);
		}
	}
	unordered_map<unsigned long, MapPoint*>::iterator it;
	for( int i = 0; i < delete_list.size(); i++)
	{
		unsigned long id = delete_list.at(i);
		it = Local_Map->map_points.find(id);
		Local_Map->map_points.erase(it);
	}
	cout << "局部地图点数量：" << Local_Map->map_points.size() << endl;
};

void Map_Builder::run()
{
	int i = 2;
	cout << "开始构建地图..." << endl;
	ref_frame_id = 0;
	for(i; i < Global_Map->key_frames.size(); i++)
	//for(i; i < 10; i++)
	{
		cout << "重建点数量：" << Global_Map->map_points.size() << endl;
		ref_frame_id = i-1;
		cur_frame_id = i;
		//update_local_map();
		unordered_map<unsigned long, Frame*>::iterator it1, it2;
		//it1 = Global_Map->key_frames.find(cur_frame_id-1);
		it1 = Global_Map->key_frames.find(ref_frame_id);
		it2 = Global_Map->key_frames.find(cur_frame_id);
		Solve_2d2d(it1, it2);
		cout << "第" << it2->second->id << "帧处理完毕 " << endl;
		//Frame* cur_frame = (Global_Map->key_frames.find(cur_frame_id))->second;
	}
	return;
};
	
void Map_Builder::pose_estimation_2d2d ( 
			std::vector<KeyPoint> keypoints_1,
                        std::vector<KeyPoint> keypoints_2,
                        std::vector< DMatch > matches,
                        Mat& R, Mat& t )
{
    // 相机内参，写在congif_hc.h
    Mat K = ( Mat_<double> ( 3,3 ) << 
		camera_Intrinsics_FX, 0, camera_Intrinsics_CX, 
	        0, camera_Intrinsics_FY, camera_Intrinsics_CY, 
	        0, 0, 1 );
    
    //匹配点转换为vector<Point2f>的形式
    points1.clear();
    points2.clear();

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //基础矩阵
   Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
    //cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //本质矩阵
    Point2d principal_point ( K.at<double>(0,2), K.at<double>(1,2) );	//相机光心
    double focal_length = K.at<double>(1,1);	//相机焦距
    
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //单应矩阵
    
    Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;
    
    
    //本质矩阵 恢复旋转和平移
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R ： "<<endl<<R<<endl;
    cout<<"t ： "<<endl<<t<<endl;
    
}	

void Map_Builder::find_feature_matches ( 
			const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches,
			    Mat& descriptors_1,
			    Mat& descriptors_2
						)
{
   
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //Oriented FAST 
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //BRIEF描述子匹配，Hamming 距离
    vector<DMatch> match;
    
    matcher->match ( descriptors_1, descriptors_2, match );

    //匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    //printf ( "Max dist : %f \n", max_dist );
    //printf ( "Min dist : %f \n", min_dist );

    //大于两倍max(30,最小距离)，视为误匹配
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

void Map_Builder::triangulate ( 
    const vector< KeyPoint >& keypoint_1, 
    const vector< KeyPoint >& keypoint_2, 
    const std::vector< DMatch >& matches,
    const Mat& Tcw1, const Mat& Tcw2, 
    const Mat& R, const Mat& t,
    vector< Point3d >& points ,
    Mat&  descriptors_1,
    vector<Mat> & descriptors
)
{
    Mat T1 = (Mat_<double> (3,4) <<
        Tcw1.at<double>(0,0), Tcw1.at<double>(0,1), Tcw1.at<double>(0,2), Tcw1.at<double>(0,3),
        Tcw1.at<double>(1,0), Tcw1.at<double>(1,1), Tcw1.at<double>(1,2), Tcw1.at<double>(1,3),
        Tcw1.at<double>(2,0), Tcw1.at<double>(2,1), Tcw1.at<double>(2,2), Tcw1.at<double>(2,3)
	    
   );
    Mat T2 = (Mat_<double> (3,4) <<
        Tcw2.at<double>(0,0), Tcw2.at<double>(0,1), Tcw2.at<double>(0,2), Tcw2.at<double>(0,3),
        Tcw2.at<double>(1,0), Tcw2.at<double>(1,1), Tcw2.at<double>(1,2), Tcw2.at<double>(1,3),
        Tcw2.at<double>(2,0), Tcw2.at<double>(2,1), Tcw2.at<double>(2,2), Tcw2.at<double>(2,3)
    );
    
    //config_hc.h
    Mat K = ( Mat_<double> ( 3,3 ) << 
		camera_Intrinsics_FX, 0, camera_Intrinsics_CX, 
	        0, camera_Intrinsics_FY, camera_Intrinsics_CY, 
	        0, 0, 1 );
    pts_1.clear();
    pts_2.clear();
    //Mat pts_1, pts_2;
    int i = 0;
    //对极约束
    Mat t_x = ( Mat_<double> ( 3,3 ) <<
                0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
                t.at<double> ( 2,0 ),      0,                      -t.at<double> ( 0,0 ),
                -t.at<double> ( 1.0 ),     t.at<double> ( 0,0 ),      0 );
    
    for ( DMatch m:matches )
    {
	Point2d pt1 = pixel2cam ( keypoint_1[ m.queryIdx ].pt, K );
        Mat y1 = ( Mat_<double> ( 3,1 ) << pt1.x, pt1.y, 1 );
        Point2d pt2 = pixel2cam ( keypoint_2[ m.trainIdx ].pt, K );
        Mat y2 = ( Mat_<double> ( 3,1 ) << pt2.x, pt2.y, 1 );
        Mat d = y2.t() * t_x * R * y1;
	if(abs(d.at<double>(0,0)) <0.1)
	{
        pts_1.push_back ( pt1 );
        pts_2.push_back ( pt2 );
	descriptors.push_back(descriptors_1.row(m.queryIdx).clone());
	}
	else cout << "Point droped" << endl;
       // pts_1.push_back ( pixel2cam( keypoint_1[m.queryIdx].pt, K) );
       // pts_2.push_back ( pixel2cam( keypoint_2[m.trainIdx].pt, K) );
	//descriptors.push_back(descriptors_1.row(m.queryIdx).clone());
    }
    
    Mat pts_4d;
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );
    
    // 2非齐次坐标
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        Mat x = pts_4d.col(i);
        x /= x.at<double>(3,0); // 归一化
        Point3d p (
            x.at<double>(0,0), 
            x.at<double>(1,0), 
            x.at<double>(2,0) 
        );
        points.push_back( p );
    }
}
Point2d Map_Builder::pixel2cam(const Point2d& p, const Mat& K)
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}
}