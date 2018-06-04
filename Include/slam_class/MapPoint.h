#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "../include_libs.h"
#include"Frame.h"

namespace slam_class
{
    
class MapPoint
{
public:
    
		unsigned long      id;
		bool        is_good;     	 // wheter a good point 
		Vector3d    pos;       	// Position in world
		Vector3d    norm;       	// Normal of viewing direction 
		Mat         descriptor;  	// Descriptor for matching 
    
		// key-frames that can observe this point 
		list<Frame*>    observed_frames;
    
		int         matched_times;     	// being an inliner in pose estimation
		int         visible_times;     	// being visible in current frame 
    
		MapPoint()
		: id(-1), pos(Vector3d(0,0,0)), norm(Vector3d(0,0,0)), is_good(true), visible_times(0), matched_times(0)
		{ };
		
		MapPoint( 
			unsigned long _id, 
			const Vector3d& _pos, 
			const Vector3d& _norm, 
			Frame* _frame= NULL, 
			const Mat& _descriptor=Mat() 
		)
		: id(_id), pos(_pos), norm(_norm), is_good(true), visible_times(1), matched_times(1), descriptor(_descriptor)
		{
			if(_frame != NULL) observed_frames.push_back(_frame);
		};
    
		inline cv::Point3f getPositionCV() const {
			return cv::Point3f( pos(0,0), pos(1,0), pos(2,0) );
		};
};

}

#endif 