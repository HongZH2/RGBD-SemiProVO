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
#ifndef MODEL_H
#define MODEL_H
#include "FastVO/common_include.h"
namespace FastVO
{
class Model
{
public:
typedef shared_ptr<Model> Ptr;
unsigned long model_maximun_; // the maximum scale of Model
SE3 T_c_w_; // transfromation from world to camera
pcl::PointCloud<pcl::PointXYZ>::Ptr model_pointcloud_; //Model Poind Cloud
vector<Eigen::Matrix3d> covar_model_pc_; //Model Point Cloud with the corvariance matrix
public:
Model();
void updateModel( pcl::PointCloud<pcl::PointXYZ>::Ptr cur_pointcloud,  vector<Eigen::Matrix3d> & covariance_fr , vector<Eigen::Vector3d> & final_indices );
//void insertModelPoint( FeaturePoint::Ptr feature_point);
//void updateModelCov();
//void ModelReset( );
};
}
#endif // MODEL_H
