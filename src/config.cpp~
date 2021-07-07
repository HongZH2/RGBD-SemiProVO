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
#include "FastVO/config.h"
namespace FastVO
{
void Config::setParameterFile( const std::string& filename )
{
if ( config_ == nullptr )
config_ = shared_ptr<Config>(new Config);
config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
if ( config_->file_.isOpened() == false )
{
std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
config_->file_.release();
return;
}
}
Config::~Config()
{
if ( file_.isOpened() )
file_.release();
}
shared_ptr<Config> Config::config_ = nullptr;
}
