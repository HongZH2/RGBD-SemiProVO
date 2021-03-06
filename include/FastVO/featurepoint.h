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
#ifndef FEATUREPOINT_H
#define FEATUREPOINT_H
#include "FastVO/frame.h"
#include "FastVO/camera.h"
namespace FastVO
{
    class FeaturePoint
    {
    public:
        typedef shared_ptr<FeaturePoint> Ptr; 
        enum PtState {OLD=-1, UPDATE=0, NEW} ;
        PtState ptstate_; // the state of points   
        unsigned long id_; //ID
        Vector3d pts_3d_; //3d points in the reference frame
        double variance_uu_;  //the variance value of feature points which are set by "default.yaml"
        double variance_vv_;
        //Vector3d pts_3d_means_; //the mean value of feature points
        Eigen::Matrix3d pt_covariance_; // the corvariance matrix of the noisy measurement
        bool skip_cur_point_ = false;
    public:
        // functions
        FeaturePoint();
        FeaturePoint( unsigned long id, Vector3d pts_3d );
        ~FeaturePoint();
        void calculateVarMat(  cv::Point2d keypoint_curr, Camera::Ptr camera, double depth, bool skip_cur_point_);
        static FeaturePoint::Ptr createFeaturePoint( unsigned long factory_id );
    };
}
#endif // FEATUREPOINT_H
