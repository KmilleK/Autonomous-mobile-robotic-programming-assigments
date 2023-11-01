////////////////////////////////////////////////////////////
// Common setting and basic inclusion information
//
#pragma once

#ifndef BA_COMMON_H
#define BA_COMMON_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


// for Eigen
#include <Eigen/Dense>

// for Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>


// typedefs for Eigen
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 3, 4> Mat34;
typedef Eigen::Matrix<double, 4, 4> Mat44;

typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;

// typedefs for Sophus
typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

using namespace std;

#endif