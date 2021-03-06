/*
Copyright 2018. All rights reserved.
Shanghai Jiao Tong University, China

Authors: Hong Zhang

'An Improved Fast Visual Odometry' is free software; you can redistribute it
and/or modify it under theterms of the GNU General Public License as published
by the Free Software Foundation; either version 2 of the License, or any later version.

'An Improved Fast Visual Odometry' is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
'An Improved Fast Visual Odometry' ; if not, you can send the e-mail to:
mclarry@sjtu.edu.cn
*
*/
#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H
// define the commonly included file to avoid a long include list
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
using Eigen::Vector2d;
using Eigen::Vector3d;
// Sophus 
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;
// Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using cv::Mat;
// PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>
// std
#include <string>
#include <vector>
#include <iostream>
#include <memory>
#include <map>
#include <unordered_map>
#include <cmath>
#include <algorithm>
using namespace std;
#endif

