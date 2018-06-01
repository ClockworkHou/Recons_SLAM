//Frame
#include "../include_libs.h"
#include"Camera.h"

#ifndef FRAME_H
#define FRAME_H

namespace slam_class 
{
	class Frame
	{
	public:
		unsigned long id;
		double time_stamp;
		SE3 T_c_w;
		Camera* camera;
		Mat rgbImg;
		bool isKey;
		
		Frame();
		Frame(
			unsigned long _id,
			double _time_stamp, 
			SE3 _T_c_w, Camera* _camera,
			Mat _rgbImg,bool _isKey
		)
		: id(_id), time_stamp(_time_stamp), T_c_w(_T_c_w), camera(_camera), rgbImg(_rgbImg), is_key_frame(_isKey)
		{}
		
		void setValue(unsigned long _id,double _time_stamp, SE3 _T_c_w, Camera* _camera,Mat _rgbImg,bool _isKey);
		void setPose( const SE3& _T_c_w );
		
		Vector3d getCamCenter() const;
		
		//To check whether a point is in the frame
		bool isInFrame( const Vector3d& pt_world );
	};
}

#endif