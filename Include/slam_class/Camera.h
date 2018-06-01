//Camera
#include "../include_libs.h"

#ifndef CAMERA_H
#define CAMERA_H

namespace slam_class
{
  class Camera_Intrinsics
  {
    public:
      double fx, fy, cx, cy;
      //double depth_scale;//for RGBD camera
      
      Camera_Intrinsics();
     
      void setValue (double _fx, double _fy, double _cx, double _cy)
      {fx = _fx;fy = _fy; cx = _cx; cy = _cy;}
      
  };
  class Camera
  {
  public:
	  Camera_Intrinsics  intrinsics;
	  
	  Camera();
	  
	  void setIntrinsics(double _fx, double _fy, double _cx, double _cy);
	  
	  Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
	  Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
	  Vector2d camera2pixel( const Vector3d& p_c );
	  Vector3d pixel2camera( const Vector2d& p_p, double depth=1 ); 
	  Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 );
	  Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );
  };
}

#endif // CAMERA_H