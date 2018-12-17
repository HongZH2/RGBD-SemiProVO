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
#ifndef ICP_FR2MODEL_H
#define ICP_FR2MODEL_H
#include "FastVO/common_include.h"
#include <pcl/kdtree/kdtree_flann.h>
namespace FastVO
{
    class ICP_fr2model
    {
    public:
        typedef std::shared_ptr<ICP_fr2model> Ptr;
        SE3 Trans_c_w_; // the result of ICP algorithm
        int max_iteration_; // the maximum number of  iterations 
        int k_closetpoint_; 
        double tolerance_t_;
        double tolerance_r_;
        vector<Eigen::Vector3d> final_corresponding_indices_;
    public:
        ICP_fr2model();
        ~ICP_fr2model();
        void poseEstimation_SICP( pcl::PointCloud<pcl::PointXYZ>::Ptr model,  pcl::PointCloud<pcl::PointXYZ>::Ptr transpc , pcl::PointCloud<pcl::PointXYZ>::Ptr source, vector<Eigen::Matrix3d> & model_cov, vector<Eigen::Matrix3d> & source_cov, vector<Eigen::Matrix3d> & trans_cov , SE3 T_w_c);
        void findMahalanobisClosetpoint( int K, pcl::KdTreeFLANN<pcl::PointXYZ> & kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr source, vector<Eigen::Matrix3d> & model_cov, vector<Eigen::Matrix3d> & source_cov,  vector<Eigen::Vector3d> & id_dist_closetpts , vector<int> & indices_src, vector<int> & indices_mdl );
        void findInliersRatio(vector<Eigen::Vector3d> id_dist_closetpts, float & nliers_ratio);
    };
}
#endif // ICP_FR2MODEL_H
