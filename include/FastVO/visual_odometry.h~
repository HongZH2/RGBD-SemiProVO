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
#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H
#include "FastVO/common_include.h"
#include "FastVO/model.h"
#include "FastVO/frame.h"
#include "FastVO/icp_fr2model.h"
#include "FastVO/featurepoint.h"
namespace FastVO
{
class VisualOdometry
{
public:
typedef shared_ptr<VisualOdometry> Ptr;
enum VOState {
INITIALIZING=-1,
OK=0,
LOST
};
VOState state_; // current VO status
Model::Ptr currentmodel_; //model with all frames and model points which is also the reference frame
Frame::Ptr currentframe_; //curent frame
ICP_fr2model::Ptr ICP_fr2model_; // the pose estimation algorithm

vector<cv::Point3d> pts_3d_ref_; // 3d points in reference frame
vector<Eigen::Matrix3d> pts_3d_corv_ref_;  // the corvariance matrix of 3d points in the camera frame
vector<cv::Point3d> pts_3d_camera_; //3d points in the camera frame
vector<Eigen::Matrix3d> pts_3d_corv_camera_;  // the corvariance matrix of 3d points in the camera frame
vector<cv::Point2d> keypoints_curr_; // keypoints in current frame
SE3 T_c_r_estimated_; // the estimated pose of current frame
// Transformed pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr transpc;
vector<Eigen::Matrix3d> trans_cov;

int num_inliers_; // number of inlier features in icp
int num_lost_; // number of lost times
// parameters for  detecting Shi_Tomasi features
int minnum_features_; // the minium number of features
int maxnum_features_; // the maximum number of features
double quality_Level_; // the quality level of Shi-Tomasi features
double min_Distance_; // the minium distance between two features 
int blockSize_; 
bool useHarrisDetector_; // 
double Shi_Tomasi_k_; //
public: //functions
VisualOdometry();
~VisualOdometry();
bool AddFrame( Frame::Ptr frame); //add a new frame
protected:
// inner operation
void extractKeyPoints();
void generatePointcloud();
void poseEstimationfr2model();
bool checkEstimatedPose();
void addCurrentFrame();
//void findClosetpoint(  unsigned long id , FeaturePoint::Ptr feature_point );
};
}
#endif // VISUALODOMETRY_H
