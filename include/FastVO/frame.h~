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
#ifndef FRAME_H
#define FRAME_H
#include "FastVO/common_include.h"
#include "FastVO/camera.h"
namespace FastVO
{
class Frame
{
public:
typedef std::shared_ptr<Frame> Ptr;
unsigned long id_; //id of Current Frame 
double time_stamp_; 
SE3 T_c_w_; // transfromation from world to camera
Camera::Ptr camera_; //Pinhole RGBD Camera model
Mat rgbimg_, depthimg_; // RGBD image
pcl::PointCloud<pcl::PointXYZ>::Ptr cur_pointcloud_; //Current Poind Cloud
vector<Eigen::Matrix3d> covar_frame_pc_; //Model Point Cloud with the corvariance matrix
public:
Frame();
Frame ( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat rgbimg=Mat(), Mat depthimg=Mat() );
~Frame();
// factory function
static Frame::Ptr createFrame();
// find the depth in depth map
double findDepth( cv::Point2d kp );
// Get Camera Center
Vector3d getCamCenter() const;
// void insertFrameCor( FeaturePoint::Ptr feature_point);
// generate the current pointcloud
//pcl::PointCloud<pcl::PointXYZRGB> & GeneratePointCloud( const cv::KeyPoint& kp );
};
}
#endif // FRAME_H
