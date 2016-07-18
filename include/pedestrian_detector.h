#pragma once

#include "video_stream.h"
#include "json/json.h"
#include "bounding_box.h"

class PedestrianDetector {

public:
    PedestrianDetector(const std::string &config_filename);
    void runDetection();


private:
    cv::Mat_<float> P;
    void setPMatrix();
    void plotLocations(cv::Mat &in_out_frame, 
                        const std::vector<cv::Point> &locations,
                        const std::vector<double> weights = std::vector<double>(),
                        const cv::Size det_size = cv::Size(64,128));
    std::vector<BoundingBox> generateCandidatesWCalibration(int imageHeight, int imageWidth, double *maxHeight,
                            float meanHeight = 1800, float stdHeight = 100, float factorStdHeight = 2); 

    Json::Value config;
    cv::HOGDescriptor hog;
};