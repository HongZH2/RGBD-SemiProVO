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
#include "FastVO/model.h"
#include "FastVO/config.h"
namespace FastVO
{
Model::Model(): T_c_w_( SE3() ), model_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    model_maximun_ = Config::get<int>("model.maximun");
}
void Model::updateModel( pcl::PointCloud<pcl::PointXYZ>::Ptr trans_pc,  vector<Eigen::Matrix3d> & trans_cov, vector<Eigen::Vector3d> & final_indices)
{
     Eigen::Vector3d pose_k;
     Eigen::Matrix3d covariance_k;
     Eigen::Vector3d pose_estimated;
     Eigen::Matrix3d covariance_estimated;
     Eigen::Vector3d pose_model;
     Eigen::Matrix3d covariance_model;
     Eigen::Matrix3d updata_k;
     pcl::PointCloud<pcl::PointXYZ>::Ptr estimated_model_pc (new  pcl::PointCloud<pcl::PointXYZ>) ;
     pcl::PointCloud<pcl::PointXYZ>::Ptr new_model_pc (new  pcl::PointCloud<pcl::PointXYZ>) ;
     pcl::PointCloud<pcl::PointXYZ>::Ptr old_model_pc (new  pcl::PointCloud<pcl::PointXYZ>) ;
     vector<Eigen::Matrix3d> estimated_cov_model_pc;
     vector<Eigen::Matrix3d> new_cov_model_pc;
     vector<Eigen::Matrix3d> old_cov_model_pc;
     vector<double> inliers_indices;
     vector<double> outliers_indices;
     vector<double> model_indices;
     vector<double> result_indices;
     if ( trans_pc->size() != trans_cov.size() && final_indices.size() != trans_cov.size() )
     {
         cerr<<"The size of covariance and transformed point cloud is not equal!!";
     }
     else
     {
         cout<<"----------------Start The Update Of Model----------------"<<endl;
         for ( size_t i=0; i < final_indices.size(); i++)
         {
             pose_k << trans_pc->points[ final_indices[i](0) ].x , trans_pc->points[ final_indices[i](0) ].y ,trans_pc->points[ final_indices[i](0) ].z;
             covariance_k << trans_cov[final_indices[i](0)];
             pose_model << model_pointcloud_->points[ final_indices[i](1) ].x,model_pointcloud_->points[ final_indices[i](1) ].y,model_pointcloud_->points[ final_indices[i](1)].z;
             covariance_model << covar_model_pc_[final_indices[i](1)];
             if( final_indices[i](2) <= 0.001)  // a  theshold to define inliers or outliers
             {
                 // Kalman Filter
                 updata_k = covariance_model * (covariance_model + covariance_k).inverse();
                 pose_estimated = pose_model + updata_k * ( pose_k - pose_model);
                 covariance_estimated = ( Eigen::Matrix3d::Identity() - updata_k) * covariance_model;
                 // Store the estimated results in a Point Cloud
                 pcl::PointXYZ point_estimated_pc(pose_estimated(0), pose_estimated(1), pose_estimated(2));
                // cout<<"pose_k:"<<trans_pc->points[ final_indices[i](0) ]<<endl;
           //     cout<<"point_estimated_pc::"<<point_estimated_pc<<endl;
                // cout<<"pose_model:"<<model_pointcloud_->points[ final_indices[i](1)]<<endl;
                 estimated_model_pc->push_back(point_estimated_pc);
                 estimated_cov_model_pc.push_back(covariance_estimated);
                 inliers_indices.push_back( final_indices[i](1));
             }
             else
             {
                 pcl::PointXYZ point_new(pose_k(0), pose_k(1), pose_k(2));
                 new_model_pc->push_back(point_new);
                 new_cov_model_pc.push_back(covariance_k);
            }
         }
         // Find the outliers in the model
          for(size_t c= 0; c<model_pointcloud_->size() ;c++)
          {
              model_indices.push_back(c);
         }
         sort(inliers_indices.begin(), inliers_indices.end());
         set_intersection(model_indices.begin(), model_indices.end(),inliers_indices.begin(), inliers_indices.end(), back_inserter(result_indices) );
         set_difference( model_indices.begin(), model_indices.end(),result_indices.begin(), result_indices.end(), back_inserter(outliers_indices) );
         cout<<"model_indices size:::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<model_indices.size()<<endl;
         cout<<"inliers_indices size:::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<inliers_indices.size()<<endl;
         cout<<"outliers_indices size:::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<outliers_indices.size()<<endl;
          cout<<"----------------Delete the oldest point----------------"<<endl;
          if( (estimated_model_pc->size() + new_model_pc->size()) >= model_maximun_ )
          {
              cerr<<"model_maximun is adjusted!"<<endl;
              model_pointcloud_->clear();
              * model_pointcloud_  = (*estimated_model_pc+ *new_model_pc);
              covar_model_pc_.clear();
              covar_model_pc_.resize(estimated_model_pc->size() + new_model_pc->size());
              for( size_t i= 0; i < (estimated_model_pc->size() + new_model_pc->size() ); ++i)
              {
                  if( i < estimated_model_pc->size() )
                  {
                      covar_model_pc_[i] = estimated_cov_model_pc[i];
                  }
                  else
                  {
                          covar_model_pc_[i] = new_cov_model_pc[ i - estimated_model_pc->size() ];
                   }
                }
          }
          else
          {
          if( (estimated_model_pc->size() + new_model_pc->size() +outliers_indices.size() ) <= model_maximun_)
          {
              for( size_t j=0; j < outliers_indices.size(); j++)
              {
                  old_model_pc->push_back( model_pointcloud_->points[outliers_indices[j]] );
                  old_cov_model_pc.push_back( covar_model_pc_[outliers_indices[j]] );
              }
          }
          else
          {
              cout<<"@@@@@@@@@@@@@@The scale of Model has been limited!@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
                for( size_t j = 1; (estimated_model_pc->size() + new_model_pc->size() + j) <= model_maximun_ ; j++)
                {
                      old_model_pc->push_back( model_pointcloud_->points[outliers_indices[ outliers_indices.size() - j  ]] );
                      old_cov_model_pc.push_back( covar_model_pc_[outliers_indices[ outliers_indices.size() - j ]] );
                }
          }
          model_pointcloud_->clear();
          cout<<"old_model_pc size:::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<old_model_pc->size()<<endl;
          cout<<"estimated_model_pc size::::::::::::::::::::::::::::::::::::::::::::::"<<estimated_model_pc->size()<<endl;
          cout<<"new_model_pc size::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<new_model_pc->size()<<endl;
          * model_pointcloud_  = * old_model_pc + (*estimated_model_pc+ *new_model_pc);
          cout<<"model_pointcloud_ size::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<model_pointcloud_->size()<<endl;
          covar_model_pc_.clear();
          covar_model_pc_.resize(estimated_model_pc->size() + new_model_pc->size() + old_model_pc->size());
          for( size_t i= 0; i < (estimated_model_pc->size() + new_model_pc->size() + old_model_pc->size()); ++i)
          {
              if( i < old_model_pc->size() )
              {
                  covar_model_pc_[i] = old_cov_model_pc[i];
              }
              else
              {
                  if(i < (estimated_model_pc->size() + old_model_pc->size()))
                  {
                       covar_model_pc_[i] = estimated_cov_model_pc[ i - old_model_pc->size() ];
                  }
                  else
                  {
                      covar_model_pc_[i] = new_cov_model_pc[ i - estimated_model_pc->size()- old_model_pc->size() ];
                  }
              }
          } 
          }
     }
}
}

