
#include "../Include/slam_class/Frame.h"

namespace slam_class
{
	
void Frame::setValue(unsigned long _id,double _time_stamp, SE3 _T_c_w, Camera* _camera,Mat _rgbImg,bool _isKey)
{
		id = _id;
		time_stamp = _time_stamp;
		T_c_w = _T_c_w;
		camera = _camera;
		rgbImg = _rgbImg;
		isKey = _isKey;
}

void Frame::setPose ( const SE3& _T_c_w )
{
		T_c_w = _T_c_w;
}

Vector3d Frame::getCamCenter() const
{
		return T_c_w.inverse().translation();
}

bool Frame::isInFrame ( const Vector3d& pt_world )
{
		Vector3d p_cam = camera_->world2camera( pt_world, T_c_w );
		 
		if ( p_cam(2,0) < 0 ) return false;
		
		Vector2d pixel = camera_->world2pixel( pt_world, T_c_w );
		
		return (pixel(0,0)>0 
		       && pixel(1,0)>0 
		       && pixel(0,0)<rgbImg.cols 
		       && pixel(1,0)<rgbImg.rows);
}

}