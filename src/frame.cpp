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
#include "FastVO/frame.h"
namespace FastVO
{
Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr), cur_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
}
Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera,Mat rgbimg, Mat depthimg )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), rgbimg_(rgbimg), depthimg_(depthimg), cur_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
}
Frame::~Frame()
{
} 
Frame::Ptr Frame::createFrame()
{
static long factory_id = 0;
return Frame::Ptr( new Frame(factory_id++) );
}
double Frame::findDepth ( cv::Point2d kp)
{
int x = cvRound(kp.x);
int y = cvRound(kp.y);
ushort d = depthimg_.ptr<ushort>(y)[x];
if ( d==0 )
return d;
else
return double(d)/camera_->depthscale_;
}
Vector3d Frame::getCamCenter() const
{
return T_c_w_.inverse().translation();
}
//void Frame::insertFrameCor( FeaturePoint::Ptr feature_point )
//{
   // corvar_framepc_.insert( make_pair(feature_point->id_, feature_point) );
   //  feature_point->ptstate_ =  feature_point->NEW;
//}
}

