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

#include "FastVO/camera.h"
#include <FastVO/config.h>
namespace FastVO
{
Camera::Camera() 
{
     fx_ = Config::get<float>("camera.fx");
     fy_ = Config::get<float>("camera.fy");
     cx_ = Config::get<float>("camera.cx");
     cy_ = Config::get<float>("camera.cy");
     depthscale_ = Config::get<float>("camera.depth_scale");
}
Vector3d Camera::world2camera ( const Vector3d& p_w, const SE3& T_c_w )
{
return T_c_w*p_w;
}
Vector3d Camera::camera2world ( const Vector3d& p_c, const SE3& T_c_w )
{
return T_c_w.inverse() *p_c;
}
Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth )
{
return Vector3d (
( p_p ( 0,0 )-cx_ ) *depth/fx_,
( p_p ( 1,0 )-cy_ ) *depth/fy_,
depth
);
}
}
