/*
Copyright 2018. All rights reserved.
Shanghai Jiao Tong University, China

*/
#include "FastVO/icp_fr2model.h"
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <FastVO/config.h>
namespace FastVO
{
    ICP_fr2model::ICP_fr2model()
    {
        max_iteration_ = Config::get<int> ("maxnum.iteration");
        k_closetpoint_ = Config::get<int> ("closetpoint.k" );
        tolerance_t_ = Config::get<double> ("tolerance.t" );
        tolerance_r_ = Config::get<double> ("tolerance.r" );
    }
    ICP_fr2model::~ICP_fr2model()
    {
    }
    bool SortFunction ( Eigen::Vector3d i, Eigen::Vector3d  j) 
    { 
        return (i(2) < j(2)); 
    }
    void transformCovariance ( vector<Eigen::Matrix3d> & source_cov, vector<Eigen::Matrix3d> & trans_cov,  Eigen::Matrix4d & transformation_matrix)
    {
        Eigen::Matrix3d T_c_r_R;
        Eigen::Vector3d T_c_r_Trans;
        T_c_r_R = transformation_matrix.block<3,3>(0,0);
        T_c_r_Trans = transformation_matrix.block<3,1>(0,3);
        vector<Eigen::Matrix3d> copy_cov( source_cov ) ;
        trans_cov. clear();
        for (size_t i = 0; i < copy_cov.size(); ++i)
        {
           trans_cov.push_back( T_c_r_R * copy_cov[i] * T_c_r_R.transpose() );
        }
    }
    void getChangesInTransformation (double & dR, double & dT, int num_iter, vector<Eigen::Matrix4d> &  record_trans)
    {
        dR = 0;
        dT = 0;
        int count = 0;
        double rdiff;
        double tdiff;
        for( int k= max(num_iter-2, 1); k!=num_iter; k++)
        {
            SE3 Trans_k = SE3( SO3(record_trans[k].block<3,3>(0,0)), record_trans[k].block<3,1>(0,3));
            SE3 Trans_k_1 = SE3( SO3(record_trans[k+1].block<3,3>(0,0)), record_trans[k+1].block<3,1>(0,3));
            Eigen::Matrix<double,1, 4> qs_k ( Trans_k.unit_quaternion().w(), Trans_k.unit_quaternion().x(), Trans_k.unit_quaternion().y(), Trans_k.unit_quaternion().z()) ;
            Eigen::Matrix<double,1, 4> qs_k_1 ( Trans_k_1.unit_quaternion().w(), Trans_k_1.unit_quaternion().x(), Trans_k_1.unit_quaternion().y(), Trans_k_1.unit_quaternion().z()) ;
            // Rotation difference in radians
            rdiff = acos( qs_k.dot(qs_k_1) / qs_k.norm() / qs_k_1.norm() );
            // Euclidean difference
            tdiff = ( Trans_k.translation() - Trans_k_1.translation() ).norm() ;
            dR = dR + rdiff;
            dT = dT + tdiff;
            count = count + 1;
        }
         dT = dT/count;
         dR = dR/count;
    }
    void Golden_Section_Search( double inlierRatio_low, double inlierRatio_high, double & epison1,  double & epison2)
    {
        double fraction_w = 0.38197;
        epison1 = inlierRatio_low+fraction_w*(inlierRatio_high-inlierRatio_low);
        epison2 = inlierRatio_high-fraction_w*(inlierRatio_high-inlierRatio_low);
    }
    double Calculate_Fai( vector<int> & inliers_indices_src, vector<int> & inliers_indices_mdl, vector<double> & inliers_indices_dist, int pc_count, float inlierRatio )
    {
         int upperbound = cvRound(inlierRatio*pc_count);
        double sum_indices = 0;
        for( auto  iter = inliers_indices_dist.begin(); iter != ( inliers_indices_dist.begin() + upperbound ) ; iter++)
        {
            sum_indices += *iter;
        }
        //cout<<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"<<sum_indices<<endl;
        double Err = sqrt (sum_indices /upperbound );
        return Err/inlierRatio*3;
    }
    void ICP_fr2model::findInliersRatio( vector<Eigen::Vector3d> id_dist_closetpts ,float & inliers_ratio )
    {
        double inlierRatio_low = 0.4;
        double inlierRatio_high = 1;
        double epison1;
        double epison2;
        double Fai1;
        double Fai2;
        double Fai3;
        int num_iter = 8;  // the number of iteration for finding a best inliers ratio
        vector<int> inliers_indices_src;
        vector<int> inliers_indices_mdl;
        vector<double> inliers_indices_dist;
        int pc_count = id_dist_closetpts.size();
         for(int i =0; i < id_dist_closetpts.size(); i++)
         {
             inliers_indices_src.push_back(id_dist_closetpts[i](0));
             inliers_indices_mdl.push_back(id_dist_closetpts[i](1));
             inliers_indices_dist.push_back(id_dist_closetpts[i](2));
        }
        Golden_Section_Search(inlierRatio_low,inlierRatio_high,epison1,epison2);
        for( int  i =1; i <= num_iter; i++)
        {
            Fai1 = Calculate_Fai(inliers_indices_src, inliers_indices_mdl, inliers_indices_dist, pc_count, inlierRatio_low);
            Fai2 = Calculate_Fai(inliers_indices_src, inliers_indices_mdl, inliers_indices_dist, pc_count, epison1);
            if ( Fai2 > Fai1 )
            {
                 inlierRatio_high = epison1;
                 Golden_Section_Search(inlierRatio_low,inlierRatio_high,epison1,epison2);
            }
            else
            {
                Fai3 = Calculate_Fai(inliers_indices_src, inliers_indices_mdl, inliers_indices_dist, pc_count, epison2);
               if ( Fai3 > Fai2
 )
               {
                   inlierRatio_low = epison1;
                   inlierRatio_high = epison2;
                   Golden_Section_Search(inlierRatio_low,inlierRatio_high,epison1,epison2);
               }
               else
               {
                   inlierRatio_low = epison2;
                   Golden_Section_Search(inlierRatio_low,inlierRatio_high,epison1,epison2);
               }
            }
        }
        inliers_ratio = (inlierRatio_low+inlierRatio_high)/2;
    } 
   void  calculate_W( vector<Eigen::Matrix3d> movingCov,  vector<int> indices_src, vector<Eigen::Matrix3d> fixedCov, vector<int> indices_mdl, vector<Eigen::Matrix3d> & W)
   {
       Eigen::Matrix3d W1;
       Eigen::Matrix3d W2;
       W.clear();
       for( int i = 0; i<indices_src.size(); i++)
       {    
           W1 = movingCov[indices_src[i]];
           W2 = fixedCov[indices_mdl[i]];
           Eigen::EigenSolver<Eigen::Matrix3d> es((W1+W2).inverse());
           Eigen::Matrix3d D = es.pseudoEigenvalueMatrix(); 
           Eigen::Matrix3d V = es.pseudoEigenvectors(); 
           Eigen::Matrix3d D_sqrt = D.array().pow(0.5);
           W.push_back(D_sqrt*V.inverse());
       }
   }
   void point_register(Eigen::Matrix<double,3,Eigen::Dynamic>  & X, Eigen::Matrix<double,3,Eigen::Dynamic> & Y,Eigen::Matrix3d & R,Eigen::Matrix<double, 3 ,1> & t, double & FRE)
   {
       int Ncoords = X.rows();
       int Npoints = X.cols();
       int Ncoords_Y = Y.rows();
       int Npoints_Y = Y.cols();
      if (Ncoords != 3 || Ncoords_Y != 3
)
          cout<<"Error::::::::::::::::::Each argument must have exactly three rows."<<endl;
      else
      {
          if ( ( Ncoords != Ncoords_Y ) || (Npoints != Npoints_Y)
 )
              cout<<"error::::::::::::X and Y must have the same number of columns."<<endl;
         else
             if (Npoints < 3
)
                 cout<<"error::::::::::::::::X and Y must each have 3 or more columns."<<endl;
      }
      Eigen::Matrix<double,3,1> Xbar = X.rowwise().sum()/Npoints;// X centroid
      Eigen::Matrix<double,3,1> Ybar = Y.rowwise().sum()/Npoints_Y;// X centroid
      Eigen::Matrix<double,3,Eigen::Dynamic> Xtilde = X-Xbar.replicate(1,Npoints); // X relative to centroid
      Eigen::Matrix<double,3,Eigen::Dynamic> Ytilde = Y-Ybar.replicate(1,Npoints); // Y relative to centroid
      Eigen::Matrix<double,3,3> H = Xtilde*Ytilde.transpose(); // cross covariance matrix
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV );
      Eigen::Matrix<double,3,3> V = svd.matrixV(), U = svd.matrixU();  
      Eigen::Matrix<double,3,3> S = U.inverse() * H * V.transpose().inverse(); 
      Eigen::Matrix<double,3,1> diag_x;
      diag_x<<1, 1, (V*U).determinant();
      R = V * diag_x.asDiagonal() * U.transpose();
      t = Ybar - R*Xbar;
      Eigen::Matrix<double,3,Eigen::Dynamic> FREvect = R*X + t.replicate(1,Npoints) - Y;
     // FRE = FREvect.array().square().colwise().prod().rowwise().sum();
  }
  Eigen::Matrix<double,Eigen::Dynamic,6>  C_maker(Eigen::Matrix<double,3,Eigen::Dynamic> X , vector<Eigen::Matrix3d> joint_W , int N)
  {
      vector<Eigen::Matrix<double,3,1>> W1,W2,W3;
      vector<Eigen::Matrix<double,3,1>> X1,X2,X3,C1_pre,C2_pre,C3_pre,C4_pre,C5_pre,C6_pre;
      Eigen::Matrix<double,Eigen::Dynamic,1> C1,C2,C3,C4,C5,C6;
      Eigen::Matrix<double,Eigen::Dynamic,6>  final_C;
      C1.resize(3*N, 1);
      C2.resize(3*N, 1);
      C3.resize(3*N, 1);
      C4.resize(3*N, 1);
      C5.resize(3*N, 1);
      C6.resize(3*N, 1);
      final_C.resize(3*N,6);
      for (size_t i = 0; i < joint_W.size(); i++)
      {
          W1.push_back( Eigen::Matrix<double,3,1>( joint_W[i](0,0),joint_W[i](1,0),joint_W[i](2,0) ) );
          W2.push_back( Eigen::Matrix<double,3,1>( joint_W[i](0,1),joint_W[i](1,1),joint_W[i](2,1) ) );
          W3.push_back( Eigen::Matrix<double,3,1>( joint_W[i](0,2),joint_W[i](1,2),joint_W[i](2,2) ) );
      }
      for(size_t j =0; j<X.row(0).size(); j++)
      {
          X1.push_back( Eigen::Matrix<double,3,1>( X.row(0)(j), X.row(0)(j), X.row(0)(j)) );
          X2.push_back( Eigen::Matrix<double,3,1>( X.row(1)(j), X.row(1)(j), X.row(1)(j)) );
          X3.push_back( Eigen::Matrix<double,3,1>( X.row(2)(j), X.row(2)(j), X.row(2)(j)) );
      }
      for(size_t z=0; z< X.row(0).size();z++)
      {
          C1.middleRows(3*z, 3)  = -W2[z].cwiseProduct(X3[z])+W3[z].cwiseProduct(X2[z]); 
          C2.middleRows(3*z, 3) =  W1[z].cwiseProduct(X3[z])-W3[z].cwiseProduct(X1[z]); 
          C3.middleRows(3*z, 3) = -W1[z].cwiseProduct(X2[z])+W2[z].cwiseProduct(X1[z]);
          C4.middleRows(3*z, 3) = W1[z];
          C5.middleRows(3*z, 3) = W2[z];
          C6.middleRows(3*z, 3) = W3[z]; 
      }
     final_C.col(0) = C1;
     final_C.col(1) = C2;
     final_C.col(2) = C3;
     final_C.col(3) = C4;
     final_C.col(4) = C5;
     final_C.col(5) = C6;
     return final_C;
  }
 Eigen::Matrix<double,Eigen::Dynamic,1> e_maker( Eigen::Matrix<double,3,Eigen::Dynamic> X , Eigen::Matrix<double,3,Eigen::Dynamic> Y, vector<Eigen::Matrix3d> joint_W, int N )
  {
      Eigen::Matrix<double,3,Eigen::Dynamic> D = Y - X;
      vector<Eigen::Matrix<double,3,1>> W1,W2,W3;
      vector<Eigen::Matrix<double,3,1>> D1,D2,D3;
      Eigen::Matrix<double,Eigen::Dynamic,1> e;
      e.resize(3*N,1);
      for (size_t i = 0; i < joint_W.size(); i++)
      {
          W1.push_back( Eigen::Matrix<double,3,1>( joint_W[i](0,0),joint_W[i](1,0),joint_W[i](2,0) ) );
          W2.push_back( Eigen::Matrix<double,3,1>( joint_W[i](0,1),joint_W[i](1,1),joint_W[i](2,1) ) );
          W3.push_back( Eigen::Matrix<double,3,1>( joint_W[i](0,2),joint_W[i](1,2),joint_W[i](2,2) ) );
      }
      for(size_t j =0; j<X.row(0).size(); j++)
      {
          D1.push_back( Eigen::Matrix<double,3,1>( D.row(0)(j), D.row(0)(j), D.row(0)(j)) );
          D2.push_back( Eigen::Matrix<double,3,1>( D.row(1)(j), D.row(1)(j), D.row(1)(j)) );
          D3.push_back( Eigen::Matrix<double,3,1>( D.row(2)(j), D.row(2)(j), D.row(2)(j)) );
      }
     for(size_t z=0; z< X.row(0).size();z++)
      {
          e.middleRows(3*z, 3)  = W1[z].cwiseProduct(D1[z])+W2[z].cwiseProduct(D2[z])+W3[z].cwiseProduct(D3[z]);
      }
      return e;
  }
   void  anisotropicPointRegister( pcl::PointCloud<pcl::PointXYZ>::Ptr transform_pc, vector<int> inliers_indices_src , pcl::PointCloud<pcl::PointXYZ>::Ptr model, vector<int> inliers_indices_mdl, vector<Eigen::Matrix3d> W,  Eigen::Matrix4d  & transformation_matrix )
   {
         int N = inliers_indices_src.size();
         // Initial estimate of the transformation assumes anisotropy:
        Eigen::Matrix3d delta_R;
        Eigen::Matrix<double,3,3> delta_theta;
        Eigen::Matrix<double,3,1> delta_t;
        Eigen::Matrix<double,6,1> q, oldq;
        Eigen::Matrix<double,Eigen::Dynamic,1>  e;
        Eigen::Matrix<double,Eigen::Dynamic,6> C;
        Eigen::Matrix<double,3,Eigen::Dynamic> X;
        Eigen::Matrix<double,3,Eigen::Dynamic> Y;
        Eigen::Matrix<double,3,Eigen::Dynamic> Xold;
        Eigen::Matrix<double,3,Eigen::Dynamic> Xnew;
        Eigen::Matrix<double,3,Eigen::Dynamic> mean_Xold_ext;
        Eigen::Matrix<double,3,1> mean_Xold;
        Eigen::Matrix3d R;
        Eigen::Matrix<double, 3 ,1> t;
        //double final_FRE;
        double FRE = 0;
        double threshold = 1e-6;
        int n = 0; // iteration index = 0;
        double config_change = 2147483647; //define  config_change  as inifty 
        cout<<N;
         // X << transform_pc->points[inliers_indices_src[0]].x,transform_pc->points[inliers_indices_src[0]].y,transform_pc->points[inliers_indices_src[0]].z;
         X.resize(3,N);
         Y.resize(3,N);
         for( int i = 0; i<N; i++)
         {    
             //X << Eigen::Matrix<double,3,1>(transform_pc->points[inliers_indices_src[i]].x,transform_pc->points[inliers_indices_src[i]].y,transform_pc->points[inliers_indices_src[i]].z);
             X(0,i) = transform_pc->points[inliers_indices_src[i]].x;
             X(1,i) = transform_pc->points[inliers_indices_src[i]].y;
             X(2,i) = transform_pc->points[inliers_indices_src[i]].z;
             Y(0,i) = model->points[inliers_indices_mdl[i]].x;
             Y(1,i) = model->points[inliers_indices_mdl[i]].y;
             Y(2,i)= model->points[inliers_indices_mdl[i]].z;  
         }
        point_register(X,Y,R,t,FRE);
        Xold = R*X+t.replicate(1,N);
        while (config_change>threshold)
        {
            n = n+1;
           // C.resize(3*N,6);
          //  e.resize(3*N,1);
            C = C_maker(Xold,W,N);
            e = e_maker(Xold,Y,W,N); 
            q = C.householderQr().solve(e); 
            //damps oscillations
            if (n > 1)
                q = (q + oldq)/2; 
            oldq = q;
            delta_t << q(3),q(4),q(5);
            delta_t = delta_t.transpose();
            delta_theta << 1,-q(2),q(1),
                                     q(2),1,-q(0),
                                     -q(1),q(0),1;     
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(delta_theta, Eigen::ComputeFullU | Eigen::ComputeFullV );
            Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();  
            Eigen::Matrix3d Lambda = U.inverse() * delta_theta * V.transpose().inverse(); 
            delta_R = U*V.transpose();
            R = delta_R*R; // update rotation
            t = delta_R*t+delta_t; // update translation
            Xnew = R*X+t.replicate(1,N); // update moving points
            mean_Xold =  Xold.rowwise().sum()/Xold.cols();
            mean_Xold_ext = mean_Xold.replicate(1,N);
            mean_Xold_ext = (Xold - mean_Xold_ext).array().square();
            config_change = sqrt( (Xnew-Xold).array().square().sum() / mean_Xold_ext.sum() );cout<<R<<endl<<t<<endl;
            Xold = Xnew;
        }
       transformation_matrix.block<3,3>(0,0) = R;
       transformation_matrix.block<3,1>(0,3) =  t;
       transformation_matrix.block<1,4>(3,0) =  Eigen::Matrix<double,1,4>(0,0,0,1);
        /*
        for( int ii = ; ii<N; ii++ )
        {
            D = W(:,:,ii)*(Xnew(:,ii)-Y(:,ii));
            FRE(ii) = D'*D;
        }
        final_FRE = sqrt(mean(FRE));*/
   }
    void ICP_fr2model::poseEstimation_SICP( pcl::PointCloud<pcl::PointXYZ>::Ptr model,  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_pc , pcl::PointCloud<pcl::PointXYZ>::Ptr source, vector<Eigen::Matrix3d> & model_cov, vector<Eigen::Matrix3d> & source_cov, vector<Eigen::Matrix3d> & trans_cov , SE3 T_w_c)
    {
       // cout<<"--------------------ICP algorithm Starts!--------------------"<<endl;
        Eigen::Matrix4d initial_transformation_matrix ( T_w_c.matrix() );
      //  cout<<initial_transformation_matrix<<endl;
        //cout<<T_w_c.rotation_matrix()<<endl<<T_w_c.translation()<<endl;
        vector<Eigen::Vector3d>  id_dist_closetpts;
        vector<int> indices_src;
        vector<int> indices_mdl;
        vector<int> inliers_indices_src;
        vector<int> inliers_indices_mdl;
        vector<double> squaredError;
        vector<Eigen::Matrix4d> record_trans;
        Eigen::Matrix4d final_transformation_matrix;
        
        // Define  two criterias for the convergence
        vector<Eigen::Matrix3d> joint_W;
        double dR;
        double dT;
        int stopIteration = max_iteration_;
        float inliers_ratio = 0.9;
        bool AICP_Estimation = false;
        
        // Transform the source point cloud by the initial_transformation_matrix
        pcl::transformPointCloud (*source, *transform_pc, initial_transformation_matrix);
        transformCovariance(source_cov, trans_cov, initial_transformation_matrix);
        final_transformation_matrix = initial_transformation_matrix;//Eigen::Matrix4d::Identity(); 
        
        //Create the kdtree of model
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud( model );
        cout<<"-----------Create The Kdtree of Model. Success!!!-----------"<<endl;
        // Create the SVD algorithm 
        cout<< "----------Start ICP Algorithm!!!----------"<<endl;
         pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, double>::Ptr trans_svd (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, double>);
        // Start the improved ICP algorithm
        for( int num_iter=1; num_iter <= max_iteration_; ++num_iter )
        {  
            // Define a transform matrix to store the resultant matrix during the each iteration 
           Eigen::Matrix<double, 4, 4> transformation_matrix = Eigen::Matrix4d::Identity ();
            
            // Find Auto_inlierRatio at the beginning of iteration

            // Find the K closet points by Mahalanobis distance 
            findMahalanobisClosetpoint( k_closetpoint_, kdtree, model, transform_pc, model_cov, trans_cov, id_dist_closetpts, indices_src, indices_mdl);
            
            // Sort the indices by the standard of the smallest Mahalanobis
            sort(id_dist_closetpts.begin(), id_dist_closetpts.end(), SortFunction);
            inliers_indices_src.clear();
            inliers_indices_mdl.clear();
            if (num_iter == 1 )
            {
                //findInliersRatio( id_dist_closetpts , inliers_ratio );
                cout<< "-------------------------------inliers_ratio--------------------------------:"<<inliers_ratio<<endl;
            }
            for(int i =0; i < cvRound( id_dist_closetpts.size()*inliers_ratio ); i++)
            {
                inliers_indices_src.push_back(id_dist_closetpts[i](0));
                inliers_indices_mdl.push_back(id_dist_closetpts[i](1));
            }
            
            // Estimate transformation given correspondences
            if (AICP_Estimation == true)
            {
                calculate_W( trans_cov, inliers_indices_src, model_cov, inliers_indices_mdl, joint_W ); 
                anisotropicPointRegister(transform_pc, inliers_indices_src, model, inliers_indices_mdl ,joint_W, transformation_matrix);
            }
            else
            {
                trans_svd->estimateRigidTransformation(*transform_pc, inliers_indices_src ,*model, inliers_indices_mdl, transformation_matrix);  
            }
            // Update the total transformation
            final_transformation_matrix = transformation_matrix * final_transformation_matrix;
           
            //Transform the current point cloud
            pcl::transformPointCloud (*transform_pc, *transform_pc, transformation_matrix);
            transformCovariance(trans_cov, trans_cov, transformation_matrix); 
            record_trans.push_back(transformation_matrix);
            
            // Check convergence  
            if( num_iter > 1)
            { 
                stopIteration = num_iter;
                getChangesInTransformation( dR, dT, num_iter, record_trans);
                if ( dR <= tolerance_r_ && dT <= tolerance_t_)
                    break;
            }
        }
         cout<< "----------ICP stopIteration: "<< stopIteration<<"----------"<<endl;
        // store  the estimated  transformation matrix
        Eigen::Matrix3d T_c_r_R;
        Eigen::Vector3d T_c_r_Trans;
        T_c_r_R = final_transformation_matrix.block<3,3>(0,0);
        T_c_r_Trans = final_transformation_matrix.block<3,1>(0,3);
        Trans_c_w_ = SE3( SO3(T_c_r_R), T_c_r_Trans);
        cout << T_c_r_R << "    "<<T_c_r_Trans<<endl;
        // store the corresponding indices
         final_corresponding_indices_.swap(id_dist_closetpts);
       
    }
    void  ICP_fr2model::findMahalanobisClosetpoint ( int K, pcl::KdTreeFLANN<pcl::PointXYZ> & kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr source, vector<Eigen::Matrix3d> & model_cov, vector<Eigen::Matrix3d> & source_cov,  vector<Eigen::Vector3d> & id_dist_closetpts , vector<int> & indices_src, vector<int> & indices_mdl  )
    {
        indices_src.clear();
        indices_mdl.clear();
        id_dist_closetpts.clear();
        vector<int> pointIdxNKNSearch(K);
        vector<float> pointNKNSquaredDistance(K);
        vector<double> pointNKNMahalanobisDistance(K);
        // find the K closet point
        for( int j = 0;  j < source->size(); ++j)
        {
            pcl::PointXYZ search_point = source->points[j];
            Eigen::Vector3d source_pts;
            Eigen::Vector3d model_pts;
            double Mahalanobis_id;
            double Euclidean_distance;
            double Mahalanobis_distance;
            indices_src.push_back ( j );
            
            pointIdxNKNSearch.clear();
            pointNKNSquaredDistance.clear();
            pointNKNMahalanobisDistance.clear();
            
            if( kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance ) > 0 && pointIdxNKNSearch.size() == K && pointNKNSquaredDistance.size()==K )
            {
                for  (size_t i = 0 ; i < pointIdxNKNSearch.size() ; ++i)
                {
                   model_pts<<model->points[ pointIdxNKNSearch[i] ].x, model->points[ pointIdxNKNSearch[i] ].y, model->points[ pointIdxNKNSearch[i] ].z;
                   source_pts<< search_point.x, search_point.y, search_point.z;
                   Mahalanobis_distance = (source_pts - model_pts).transpose()*(source_cov[j]+model_cov[i])*(source_pts - model_pts);
                   pointNKNMahalanobisDistance.push_back(Mahalanobis_distance);
                }
                if( pointNKNMahalanobisDistance.size() == K)
                {
                    auto smallest_Mahalanobis = min_element(pointNKNMahalanobisDistance.begin(), pointNKNMahalanobisDistance.end());  
                    Mahalanobis_id = (double) pointIdxNKNSearch[  distance(pointNKNMahalanobisDistance.begin(), smallest_Mahalanobis) ];
                    Euclidean_distance =  (double) pointNKNSquaredDistance[ distance(pointNKNMahalanobisDistance.begin(), smallest_Mahalanobis) ];
                    id_dist_closetpts.push_back( Vector3d( (double) j, Mahalanobis_id, Euclidean_distance) );
                    indices_mdl.push_back( pointIdxNKNSearch[  distance(pointNKNMahalanobisDistance.begin(), smallest_Mahalanobis) ] );
                }
                else
                {
                    cerr<<"the size of closet Mahalanobis distance points are equal as K!!!!!!!!!!!!! ";
                }
            }
            else
            {
                cerr<<"it fails when finding the K closet point!";
            }
        }
    }
}
