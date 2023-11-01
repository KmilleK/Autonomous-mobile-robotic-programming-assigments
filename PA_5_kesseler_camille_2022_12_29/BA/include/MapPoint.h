#pragma once

#ifndef BA_MAPPOINT_H
#define BA_MAPPOINT_H

#include "Common.h"
//
namespace BA
{
    class MapPoint
    {
    public:
        unsigned long id_ = 0;          // id of landmark
        Vec3 pose_;                       // coordinates of the Map point
        
        MapPoint() {}
        MapPoint(const Vec3 &pt);
        
        Vec3 GetPose();
    };
}

#endif