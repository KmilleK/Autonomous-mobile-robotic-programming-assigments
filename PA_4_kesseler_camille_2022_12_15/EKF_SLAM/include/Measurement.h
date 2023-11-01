////////////////////////////////////////////////////////////
// Measurement data initializer

#pragma once

#ifndef EKF_SLAM_MEASUREMENT_H
#define EKF_SLAM_MEASUREMENT_H

#include "Common.h"

namespace EKF_SLAM
{
   

    class Measurement
    { 
    public:
        Measurement() {}
        Measurement(string input_file);

        vector<Sensor> data;
    };
}

#endif