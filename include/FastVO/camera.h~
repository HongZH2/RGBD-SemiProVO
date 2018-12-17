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

#ifndef CAMERA_H
#define CAMERA_H
#include "FastVO/common_include.h"
namespace FastVO
{
// pinhole RGBD camera model
class Camera 
{
public:
typedef std::shared_ptr<Camera> Ptr;
// Camera intrinsics
float fx_, fy_, cx_, cy_, depthscale_;
Camera();
Camera( float fx, float fy, float cx, float cy, float depthscale=0) :
fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depthscale_ ( depthscale )
{}
// Coordinate transform: world, camera, pixel
Vector3d world2camera( const Vector3d& point_w, const SE3& T_c_w );
Vector3d camera2world( const Vector3d& point_c, const SE3& T_c_w );
Vector3d pixel2camera( const Vector2d& p_p, double depth=1 );
};
}
#endif
