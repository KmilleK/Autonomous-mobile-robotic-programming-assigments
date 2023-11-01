#pragma once

#ifndef BA_FEATURE_H
#define BA_FEATURE_H

#include "Common.h"
#include "Frame.h"

//
namespace BA
{
    // forward declare
    struct Frame;

    class Feature
    {
    public:
        std::weak_ptr<Frame> frame_;           // observed frame
        Vec2 pt_;                           // coordinates of the feature

        Feature() {}
        Feature(const Vec2 &pt);
    };

}

#endif