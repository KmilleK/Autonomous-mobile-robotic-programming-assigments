#include "MapPoint.h"

//Map point object

namespace BA
{
    MapPoint::MapPoint(const Vec3 &pt) : pose_(pt) {}

    Vec3 MapPoint::GetPose()
    {
        return pose_;
    }
}
