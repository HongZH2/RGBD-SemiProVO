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
#ifndef CONFIG_H
#define CONFIG_H
#include "FastVO/common_include.h"
namespace FastVO
{
class Config
{
private:
static std::shared_ptr<Config> config_;
cv::FileStorage file_;
Config () {} // private constructor makes a singleton
public:
~Config(); // close the file when deconstructing
// set a new config file
static void setParameterFile( const std::string& filename );
// access the parameter values
template< typename T >
static T get( const std::string& key )
{
return T( Config::config_->file_[key] );
}
};
}
#endif // CONFIG_H
