#include "Frame.h"

//Frame object

namespace BA
{

    Frame::Frame(long id, const SE3 &pose) : id_(id), pose_(pose) {}

    SE3 Frame::GetPose()
    {
        return pose_;
    }

    void Frame::SetPose(const SE3 &pose)
    {
        pose_ = pose;
    }
    
    Vec3 Frame::world2camera(const Vec3 &p_w)
    {
        return pose_ * p_w;
    }

    Vec3 Frame::camera2world(const Vec3 &p_c)
    {
        return pose_.inverse() * p_c;
    }

    Vec2 Frame::camera2pixel(const Vec3 &p_c)
    {
        return Vec2(
            K_(0, 0) * p_c(0) / p_c(2) + K_(0, 2),
            K_(1, 1) * p_c(1) / p_c(2) + K_(1, 2));
    }

    Vec2 Frame::world2pixel(const Vec3 &p_w)
    {
        return camera2pixel(world2camera(p_w));
    }

}
