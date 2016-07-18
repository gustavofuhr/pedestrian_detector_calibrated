#include "pedestrian_detector.h"
#include <fstream>
#include <cv.h>
#include <highgui.h>

PedestrianDetector::PedestrianDetector(const std::string &config_filename) {
    std::ifstream c_file;
    // open the json file and load to the config 
    c_file.open(config_filename.c_str());
    if (!c_file.is_open())
        std::cerr << "Could not open the file!" << std::endl;
    
    c_file >> config;
    
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    setPMatrix();
}

void PedestrianDetector::plotLocations(cv::Mat &in_out_frame, 
                                        const std::vector<cv::Point> &locations,
                                        const std::vector<double> weights, /* =std::vector<double>()*/
                                        const cv::Size det_size /*= cv::Size(64,128)*/) {
    for (int i = 0; i < locations.size(); ++i) {
        cv::circle(in_out_frame, locations[i], 8, cv::Scalar(20,20,255), -1);

        // assuming this it the top-left position of the detection I can plot
        // the bounding box according to the size that is passed to the function
        cv::Rect r(locations[i], det_size);
        cv::rectangle(in_out_frame, r, cv::Scalar(20,20,255), 2);
    }

} 

void PedestrianDetector::setPMatrix() {
    const Json::Value p_list = config["detector_opts"]["P"];
    P = cv::Mat_<float>(3,4,0.0);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            P(i,j) = p_list[i*4+j].asFloat();
        }
    }
    // P = P.t();
    // now, if scale is required, scale P accordantly
    float resize_factor = config["detector_opts"]["resize_image"].asFloat();
    if (resize_factor != 1.0) {
        float scale_matrix[9] = {resize_factor, 0.0, 0.0, 0.0, resize_factor, 0.0, 0.0, 0.0, 1.0};

        cv::Mat_<float> S(3, 3, scale_matrix);
        P = S * P;
    }

    std::cout << "P " << P << std::endl;
}



std::vector<BoundingBox> PedestrianDetector::generateCandidatesWCalibration(int imageHeight, int imageWidth, double *maxHeight,
                            float meanHeight/* = 1.8m*/, float stdHeight/* = 0.1m*/, float factorStdHeight/* = 2.0*/) 
{

    // there is a set of parameters here that are hard coded, but should
    // be read from a file or something...
    cv::Mat_<float> P3 = P.col(2);

    //std::cout << "P3: " << P3 << std::endl;
    float aspectRatio = 0.41;
    float minImageHeight = 80;

    float stepHeight = 200;
    int totalCandidates = 0;

    std::vector<BoundingBox> all_candidates;
    double max_h = 0;

    // assemble the third line of the inverse of the homography
    cv::Mat_<float> H(3, 3, 0.0);
    H(0,0) = P(0,0); H(1,0) = P(1,0); H(2,0) = P(2,0);
    H(0,1) = P(0,1); H(1,1) = P(1,1); H(2,1) = P(2,1);
    H(0,2) = P(0,3); H(1,2) = P(1,3); H(2,2) = P(2,3);
    
    //std::cout << "H: " << H << std::endl;
    cv::Mat_<float> H_inv = H.inv();

    float padding = 4;

    // create foot points using the pixels of the image
    for (int u = 0; u < imageWidth; u+=padding) {
        for (int v = minImageHeight; v < imageHeight; v+=padding ) {

            float Xw = (H_inv(0,0)*u + H_inv(0,1)*v + H_inv(0,2))/(H_inv(2,0)*u + H_inv(2,1)*v + H_inv(2,2));
            float Yw = (H_inv(1,0)*u + H_inv(1,1)*v + H_inv(1,2))/(H_inv(2,0)*u + H_inv(2,1)*v + H_inv(2,2));

            // now create all_candidates at different heights
            for (float h = -stdHeight * factorStdHeight; h <= stdHeight * factorStdHeight; h+= stepHeight) {
                float wHeight = meanHeight + h;

                int head_v = (int)((Xw*P(1,0) + Yw*P(1,1) + wHeight*P(1,2) + P(1,3))/(Xw*P(2,0) + Yw*P(2,1) + wHeight*P(2,2) + P(2,3)));
                int i_height = v - head_v;

                if (i_height >= minImageHeight) {
                    int head_u = (int)((Xw*P(0,0) + Yw*P(0,1) + wHeight*P(0,2) + P(0,3))/(Xw*P(2,0) + Yw*P(2,1) + wHeight*P(2,2) + P(2,3)));

                    BoundingBox candidate;

                    int i_width = i_height*aspectRatio;

                    candidate.bb.x = (int)(((u + head_u)/2.0) - (i_width/2.0));
                    candidate.bb.y = head_v;
                    candidate.bb.width          = i_width;
                    candidate.bb.height         = i_height;
                    candidate.world_height      = wHeight;

                    if (candidate.bb.x >= 0 && candidate.bb.x+candidate.bb.width < imageWidth && 
                            candidate.bb.y >= 0 && candidate.bb.y+candidate.bb.height < imageHeight &&
                            candidate.bb.height >= minImageHeight) {
                        if (candidate.bb.height > max_h) 
                            max_h = candidate.bb.height;
                        all_candidates.push_back(candidate);
                    }
                }

            }
            
        }
    }
    
    if (maxHeight != NULL) {
        *maxHeight = max_h;    
    }

    std::cout << "Its going to return" << std::endl;
    return all_candidates;
}

/*
This function transforms the bounding boxes in pairs of scales and top-left points
*/
// std::vector<cv::Point> PedestrianDetector::computeSearchLocations(int imageHeight, int imageWidth, std::vector<cv::Rect> &bbox_candidates, float scale_pyramid) {

// }

// std::vector<cv::Mat> computeImagePyramid(cv::Mat &image, float scale_parameter, float max_scale) {

// }


void debugCandidates(cv::Mat &frame, std::vector<BoundingBox> &candidates) {
    std::cout << "number of candidates" << candidates.size() << std::endl;
    for (int i = 0; i < candidates.size(); ++i) {
        std::cout << "c " << candidates[i].bb << std::endl;
        candidates[i].plot(frame, cv::Scalar(255,0,0));
    }
    cv::imshow("debug", frame);
    cv::waitKey(0);
}

void PedestrianDetector::runDetection() {
    // open the video stream
    VideoStream *vid = read_video_stream(config["dataset"]);
    vid->open();

    std::vector<BoundingBox> candidates;

    while (!vid->has_ended()) {
        cv::Mat frame = vid->get_next_frame();

        // resize frame if needed.
        if (config["detector_opts"]["resize_image"].asDouble() > 1.0) {
            double f = config["detector_opts"]["resize_image"].asDouble();
            cv::resize(frame, frame, cv::Size(), f, f);
        }

        // if calibration is required and the candidates were not build yet
        if (config["detector_opts"]["use_calibration"].asBool()) {
            candidates = generateCandidatesWCalibration(frame.rows, frame.cols, NULL);
            std::cout << "Its going to debug..." << std::endl;
            debugCandidates(frame, candidates);
        }


        std::vector<cv::Point> found;
        std::vector<double> weights;
        hog.detect(frame, found, weights, -1);

        for (int i=0; i<found.size(); ++i)
            std::cout << " " << found[i];
        std::cout << std::endl;

        for (int i=0; i<weights.size(); ++i)
            std::cout << " " << weights[i];
        std::cout << std::endl;

        plotLocations(frame, found);

        cv::imshow("detections", frame);
        cv::waitKey();
    }
}