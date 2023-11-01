#pragma once

#ifndef BA_FRAME_H
#define BA_FRAME_H

#include "Common.h"
#include "Feature.h"
//
namespace BA
{
    // forward declare
    struct Feature;

    class Frame
    {
    public:
        unsigned long id_ = 0;          // id of this frame
        SE3 pose_;                      // Tcw Pose
        Mat33 K_;                       // Camera intrinsic parameters

        std::vector<Feature> features_;

    public:
        Frame() {}
        Frame(long id, const SE3 &pose);

        // set and get pose, thread safe
        SE3 GetPose();
        void SetPose(const SE3 &pose);

        // coordinate transform: world, camera, pixel
        Vec3 world2camera(const Vec3 &p_w);
        Vec3 camera2world(const Vec3 &p_c);
        Vec2 camera2pixel(const Vec3 &p_c);
        Vec2 world2pixel(const Vec3 &p_w);
    };

}

#endif