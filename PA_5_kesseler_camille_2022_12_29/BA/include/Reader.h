////////////////////////////////////////////////////////////
// Ground truth data initializer

#pragma once

#ifndef BA_READER_H
#define BA_READER_H

#include "Common.h"
#include "Frame.h"
#include "Feature.h"
#include "MapPoint.h"
//
namespace BA
{
    class Reader
    {
    private:
        string data_path;
    public:
        Reader() {}
        Reader(string data);

        // dataset initializer
        Mat33 init_K();
        vector<Frame> init_frame();
        vector<vector<Feature>> init_observation();
        vector<vector<int>> init_ob_ids();
        vector<MapPoint> init_point();
    };
}

#endif