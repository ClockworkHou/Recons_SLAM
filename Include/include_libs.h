#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;
//using Eigen::Vector2d;
//using Eigen::Vector3d;

#include "sophus/se3.h"
#include "sophus/so3.h"
using Sophus::SO3;
using Sophus::SE3;
#include <cv.h>
#include <highgui.h>

#include<opencv/cv.h>
#include<opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include<opencv2/opencv.hpp>  
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <chrono>

using namespace cv;
using cv::Mat;

// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <boost/timer.hpp>


//pcl
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/visualization/cloud_viewer.h>  

using namespace std; 
#endif