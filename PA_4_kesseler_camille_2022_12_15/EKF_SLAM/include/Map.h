////////////////////////////////////////////////////////////
// Ground truth data initializer

#pragma once

#ifndef EKF_SLAM_MAP_H
#define EKF_SLAM_MAP_H

#include "Common.h"

namespace EKF_SLAM
{

    struct MapPoint
    {
        long id;
        float x;
        float y;

    };

    class Map
    {
    public:
        Map() {}
        Map(string file_name);

        bool isIn(long id);
        MapPoint getMapPoint(long id);
        int addMapPoint(MapPoint landmark);

        vector<MapPoint> data;
    };
}

#endif
