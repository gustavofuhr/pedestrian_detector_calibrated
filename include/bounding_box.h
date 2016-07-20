#pragma once

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class BoundingBox
{
public:
    cv::Rect bb;
    float world_height;
    float score; // TODO: Do I need it?
    int pyramid_level;

    void plot (cv::Mat &frame, cv::Scalar color);
    
    // this was added to be able to sort BB_Array objects
    bool operator< (const BoundingBox &other) const 
    {
        //return score < other.score;
        return score > other.score;
    }

};