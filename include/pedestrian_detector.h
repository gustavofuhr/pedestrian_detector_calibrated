#pragma once

#include "video_stream.h"
#include "json/json.h"
#include "bounding_box.h"

class PedestrianDetector {

public:
    PedestrianDetector(const std::string &config_filename);
    void runDetection();


private:
    cv::Size det_size = cv::Size(64,128);
    cv::Mat_<float> P;
    void setPMatrix();
    void plotLocations(cv::Mat &in_out_frame, 
                        const std::vector<cv::Point> &locations,
                        const std::vector<double> weights = std::vector<double>());
    std::vector<BoundingBox> generateCandidatesWCalibration(int imageHeight, int imageWidth, double *maxHeight,
                            float meanHeight = 1800, float stdHeight = 100, float factorStdHeight = 2); 
    std::vector<cv::Mat> computeImagePyramid(cv::Mat &image, std::vector<float> &pyramid_scales, float scale_parameter, int n_levels = 5);

    // std::vector<cv::Point> computeSearchLocations(int imageHeight, int imageWidth, 
    //                         std::vector<cv::Rect> &bbox_candidates, float scale_pyramid);

    void associateScaleToCandidates(std::vector<BoundingBox> &candidates, const std::vector<float> &pyramid_scales);

    Json::Value config;
    cv::HOGDescriptor hog;
};