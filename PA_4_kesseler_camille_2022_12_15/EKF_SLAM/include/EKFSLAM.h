////////////////////////////////////////////////////////////
// Basic algorithm that implements EKF SLAM

#pragma once

#ifndef EKF_SLAM_EKFSLAM_H
#define EKF_SLAM_EKFSLAM_H

#include "Common.h"
#include "Map.h"

namespace EKF_SLAM
{
    struct State
    {
        Eigen::VectorXf mean;
        Eigen::MatrixXf covariance; 
    }; 

    class MY_EKF
    {
    public:
        MY_EKF() {}
        MY_EKF(const Pose &Ini_pose,const Map &Ini_map);
        //function 

        int predict( const Odometry &Movement,
            const Eigen::Matrix3f &R);
     
        int correct(const vector<Landmark> &lm,const Eigen::Matrix3f &Q);

      

        int UpdateTraj();
        
        int SaveRMSE(string file);
        int SaveMapPoints(string file); 
        int SaveTrajectory(string file); 

        int RMSE_land(Map map);

        int save_instant(int step); 

        // Memory_vector
        vector<double> RMSE; 

        Map map;
        vector<Pose> Trajectory; 
        
        //Actual Pose information 
        vector<int> Landmark_detection;

        State state; 

    };
}

#endif