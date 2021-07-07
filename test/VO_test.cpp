//---------------------------------test Visual Odometry-----------------------------------
#include <fstream>
#include <iomanip>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include "FastVO/config.h"
#include "FastVO/visual_odometry.h"
int main ( int argc, char** argv )
{
if ( argc != 2 )
{
    cout<<"usage: run_vo parameter_file"<<endl;
    return 1;
}

// ---------------------------------Set the directory of dataset---------------------------------
FastVO::Config::setParameterFile ( argv[1] );
FastVO::VisualOdometry::Ptr vo ( new FastVO::VisualOdometry );
string dataset_dir = FastVO::Config::get<string> ( "dataset_dir" );
cout<<"dataset: "<<dataset_dir<<endl;
ifstream fin ( dataset_dir+"/associate.txt" );
if ( !fin )
{
    cout<<"please generate the associate file called associate.txt!"<<endl;
    return 1;
}
vector<string> rgb_files, depth_files;
vector<double> rgb_times, depth_times;
while ( !fin.eof() )
{
    string rgb_time, rgb_file, depth_time, depth_file;
    fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
    rgb_times.push_back ( atof ( rgb_time.c_str() ) );
    depth_times.push_back ( atof ( depth_time.c_str() ) );
    rgb_files.push_back ( dataset_dir+"/"+rgb_file );
    depth_files.push_back ( dataset_dir+"/"+depth_file );
    if ( fin.good() == false )
    break;
}

// ---------------------------------define two ptr for the class Camera and the class Model---------------------------------
FastVO::Camera::Ptr camera ( new FastVO::Camera );
//FastVO::Model::Ptr pModel = FastVO::Model::createModel();
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr mpointcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//pModel->ModelPointCloud_ = mpointcloud;

// ---------------------------------visualization---------------------------------
cv::viz::Viz3d vis("Visual Odometry");
cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
vis.setViewerPose( cam_pose );
world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
vis.showWidget( "World", world_coor );
vis.showWidget( "Camera", camera_coor );
cout<<"read total "<<rgb_files.size() <<" entries"<<endl;

//---------------------------------Set the directory of the output file---------------------------------
ofstream fout ( dataset_dir +"/trajectory.txt" );
if ( !fout )
{
    cout<<"Making the trajectory file fails!"<<endl;
    return 1;
}
fout.setf(ios::fixed, ios::floatfield);  
fout<<"#This estimated trajectory refers to the path: "<<dataset_dir<<endl;

//--------------------------------- Run the Visual Odometry---------------------------------
for ( int i=0; i<rgb_files.size(); i++ )
{
    Mat color = cv::imread ( rgb_files[i], 1);
    Mat depth = cv::imread ( depth_files[i], -1 );
    if ( color.data==nullptr || depth.data==nullptr )
        break;
    
    FastVO::Frame::Ptr pFrame = FastVO::Frame::createFrame();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fpointcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pFrame->camera_ = camera;
    pFrame->rgbimg_ = color;
    pFrame->depthimg_ = depth;
    pFrame->time_stamp_ = rgb_times[i];
    
    boost::timer timer;
    
    vo->AddFrame ( pFrame);
    
    cout<<"VO costs time: "<<timer.elapsed()<<endl;
    if ( vo->state_ == FastVO::VisualOdometry::LOST )
        break;
    SE3 Tcw = pFrame->T_c_w_;
    Eigen::Quaterniond final_q;
    Eigen::Vector3d final_trans;
    final_q = Tcw.unit_quaternion();
    final_trans = Tcw.translation();

// ---------------------------------store the estimated pose---------------------------------
    fout<<pFrame->time_stamp_<<' '<<final_trans.x()<<' '<<final_trans.y()<<' '<<final_trans.z()<<' '<<final_q.x()<<' '<<final_q.y()<<' '<<final_q.z()<<' '<<final_q.w()<<endl;
    
//--------------------------------- show the map and the camera pose---------------------------------
    cv::Affine3d M(
        cv::Affine3d::Mat3(
            Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
            Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
            Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
               Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
            )
            );
    //cout<<M.rotation()<<endl<<M.translation()<<endl;
    cv::imshow("image", color );
    cv::waitKey(1);
    vis.setWidgetPose( "Camera", M);
    vis.spinOnce(1, false);
    //if(i==2) break;
}
return 0;
}


