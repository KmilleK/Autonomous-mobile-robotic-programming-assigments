////////////////////////////////////////////////////////////
// Common setting and basic inclusion information

#pragma once

#ifndef EKF_SLAM_COMMON_H
#define EKF_SLAM_COMMON_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <Eigen/Dense>

// Usefull structure for the whole EKF 
using namespace std;


    struct Pose
    {
        float x;
        float y;
        float theta;

        Eigen::Matrix3f covariance; 
    }; 

     struct Odometry
    {
        float r1;
        float t;
        float r2;
    };

    struct Landmark
    {
        long id;
        float range;
        float bearing;
    };

    struct Sensor
    {
        Odometry odom;
        vector<Landmark> lm;
    };

#endif