This is a simple RGBD Visual Odometry system based on frame-to-model method. And it's a simplized version for the test. Before using this project, please assure that you have accomplished the installment of the following 3Dparties.


  -Eigen3; 
  
  -Opencv(3.2 Recommended);
  
  -PCL(point cloud library 1.2 Required);
  
  -Sophus(old version);
  
  -G2O(Recommended);


The performance of the presented VO system is evaluated by the benchmark dataset 
TUM RGBD dataset: https://vision.in.tum.de/data/datasets/rgbd-dataset 

Besides, the video sequences like 
fr1_xyz,fr2_xyz,fr2_rpy,fr3_static_? are recommended, and the performance of testing on any other sequence is not good now. Please download at least one video sequence for the test from TUM Dataset.

In the file "config/default.yaml", please modify your path of data file adn other parameters you want to change. For example, "dataset_dir: /home/mclarry/TUM_DATASET/rgbd_dataset_freiburg1_xyz" *Please note that you must associate the RGB images and Depth images by following the instructions of TUM Dataset Website

You can easily start to use it by the following steps in your terminal. 
/********************************************************************** 

mkdir build

cd build

cmake .. 

make

bin/VO_test config/default.yaml


/*********************************************************************** 
/*********************************************************************** 

File Structure

--cmake_modules:

    FindCSparse.cmake 
    
    FindG2O.cmake 
    
--src:

    camera.cpp 
    
    config.cpp
    
    featurepoint.cpp
   
    frame.cpp 
    
    icp_fr2model.cpp 
    
    model.cpp 
    
    visual_odometry.cpp 
  
--include/FastVO: 

    common_include.h
    
    featurepoint.h 
    
    frame.h 
    
    icp_fr2model.h 
    
    model.h 
    
    visual_odometry.h 
    
--test: 

      VO_test.cpp                               // this is the main function 
      
  /***********************************************************************

the output_file.txt is located on your path of dataset. You can visualize the result by following the instructions of TUM dataset Website.

What's more, you can open the pose estimation method based on AICP algorithm in the icp_fr2model.cpp by modifying the bool variable 

＂AICP_Estimation = true;＂

Finally, feel free to contact me if you have any questions.

Email: mclarry@sjtu.edu.cn	

Name: Hong Zhang
