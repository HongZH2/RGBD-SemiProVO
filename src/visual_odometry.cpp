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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/registration/icp.h>
#include <algorithm>
#include <boost/timer.hpp>
#include "FastVO/config.h"
#include "FastVO/visual_odometry.h"
namespace FastVO
{
VisualOdometry::VisualOdometry():
state_(INITIALIZING), currentmodel_( new Model() ),ICP_fr2model_( new ICP_fr2model() ) , transpc( new pcl::PointCloud<pcl::PointXYZ>),num_inliers_ (0), num_lost_ (0)
{
minnum_features_ = Config::get<int> ("minnum_features");
maxnum_features_ = Config::get<int> ("maxnum_features");
quality_Level_ = Config::get<double> ("quality_Level");
min_Distance_ = Config::get<double> ("min_Distance");
blockSize_ = Config::get<int> ("blockSize");
useHarrisDetector_ = (bool) Config::get<int> ("useHarrisDetector"); 
Shi_Tomasi_k_ = Config::get<double> ("Shi_Tomasi_k");
}
VisualOdometry::~VisualOdometry()
{
}
void VisualOdometry::extractKeyPoints()
{
cv::Mat rgbimg_Gray;
//  transfer the rgb image into the gray image
cv::cvtColor(currentframe_->rgbimg_, rgbimg_Gray, cv::COLOR_BGR2GRAY);
cv::goodFeaturesToTrack( rgbimg_Gray, keypoints_curr_, maxnum_features_, quality_Level_, min_Distance_, Mat(), blockSize_, useHarrisDetector_, Shi_Tomasi_k_);  
if ( keypoints_curr_.size() < minnum_features_)
{
    cout<<"the scale of Shi_Tomasi features is less than "<<minnum_features_<<"!!!"<<endl;
}
/// get a copy of the raw image   
/*cv::Mat rgbimg_GrayCopy = currentframe_->rgbimg_.clone();  
/// show these features
int r = 4;  
for( int i = 0; i < keypoints_curr_.size(); i++ )  
cv::circle( rgbimg_GrayCopy, keypoints_curr_[i], r, cv::Scalar(0,255,0), -1, 8, 0 );    
cv::namedWindow(  "src", CV_WINDOW_AUTOSIZE );  
cv::imshow( "src", rgbimg_GrayCopy );     
cv::waitKey(0);*/
}
void  VisualOdometry::generatePointcloud()
{
     unsigned long factory_id = 0; //  The ID of points
     pts_3d_camera_.clear();
     pts_3d_corv_camera_.clear();
     pcl::PointXYZ pc_frame;
     bool skip_cur_point = false;
     for( size_t i = 0; i < keypoints_curr_.size(); i++ )  
     {
        // computer the position of point
        double z = currentframe_->findDepth(keypoints_curr_[i]);
        if (z > 0)
        {
           Vector3d p_cam = currentframe_->camera_->pixel2camera( Vector2d(keypoints_curr_[i].x, keypoints_curr_[i].y), z );
           FastVO::FeaturePoint::Ptr pFeaturePoint = FastVO::FeaturePoint::createFeaturePoint( factory_id++ );
           pFeaturePoint->pts_3d_ = p_cam;
           pFeaturePoint->calculateVarMat(keypoints_curr_[i], currentframe_->camera_, z , skip_cur_point); // set the means and variance of point
           if ( skip_cur_point == true )
               continue;
           pc_frame.z = z;
           pc_frame.x = p_cam(0,0);
           pc_frame.y = p_cam(1,0);
           currentframe_->cur_pointcloud_->points.push_back( pc_frame );
           currentframe_->covar_frame_pc_.push_back( pFeaturePoint->pt_covariance_ );
        }
     }
     currentframe_->cur_pointcloud_->height = 1;
     currentframe_->cur_pointcloud_->width = currentframe_->cur_pointcloud_->points.size();
     currentframe_->cur_pointcloud_->is_dense = false;
     cout<<"The scale of Shi_Tomasi features:"<< currentframe_->cur_pointcloud_->size()<<endl;
     cout<<"The scale of Shi_Tomasi features Covariance:"<< currentframe_->covar_frame_pc_.size()<<endl;
}
void VisualOdometry::poseEstimationfr2model()
{
    ICP_fr2model_->poseEstimation_SICP( currentmodel_->model_pointcloud_, transpc, currentframe_->cur_pointcloud_, currentmodel_->covar_model_pc_, currentframe_->covar_frame_pc_, trans_cov, currentmodel_->T_c_w_  ); //currentmodel_->T_c_w_ 
    T_c_r_estimated_ = ICP_fr2model_->Trans_c_w_;
}
bool VisualOdometry::checkEstimatedPose()
{
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = T_c_r_estimated_.log();
    if ( d.norm() >10000.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
}
void VisualOdometry::addCurrentFrame()
{
    currentmodel_->updateModel(transpc, trans_cov, ICP_fr2model_->final_corresponding_indices_);
}
bool VisualOdometry::AddFrame ( Frame::Ptr frame )
{
switch (state_)
{
    case INITIALIZING:
    {
        state_ = OK;
        currentframe_  = frame;
        extractKeyPoints();  
        generatePointcloud();
        // Store the current frame into model
        currentmodel_->model_pointcloud_ = currentframe_->cur_pointcloud_;
        currentmodel_->T_c_w_ = currentframe_->T_c_w_;
        currentmodel_->covar_model_pc_.swap(currentframe_->covar_frame_pc_);
        cout<<"The scale of Model Shi_Tomasi features:"<< currentmodel_->model_pointcloud_->size()<<endl;
        cout<<"The scale of Model Shi_Tomasi features Covariance:"<< currentmodel_->covar_model_pc_.size()<<endl;
        break;
    }
    case OK:
    {
       cout<<"-----------------------------------------------------------------------------NEW FRAME------------------------------------------------------------------------"<<endl;
        currentframe_  = frame;
        extractKeyPoints();
        generatePointcloud();
        
        boost::timer timer;
        poseEstimationfr2model();
        cout<<"Pose estimation costs time: "<<timer.elapsed()<<endl;
        
        if( checkEstimatedPose() ==true )
        {
            currentframe_->T_c_w_ = T_c_r_estimated_ ;
            //cout << currentframe_->T_c_w_.rotation_matrix() << "    "<<currentframe_->T_c_w_.translation() <<endl;
            currentmodel_->T_c_w_ =  currentframe_->T_c_w_;   
           // currentmodel_->model_pointcloud_ = currentframe_->cur_pointcloud_;
           // currentmodel_->covar_model_pc_.swap(currentframe_->covar_frame_pc_); 
            addCurrentFrame();
          //  cout<<"The scale of Model Shi_Tomasi features:"<< currentmodel_->model_pointcloud_->size()<<endl;
          //  cout<<"The scale of Model Shi_Tomasi features Covariance:"<< currentmodel_->covar_model_pc_.size()<<endl;
        }
        else
        {
            state_ = LOST;
        }
       // state_ = LOST;
        break;
    }
    case LOST:
    {
        cout<<"The Visual Odometry has lost!!!"<<endl;
        break;
    }
    return true;
}
}
}
