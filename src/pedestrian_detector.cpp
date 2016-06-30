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
}


void PedestrianDetector::runDetection() {
    // open the video stream
    VideoStream *vid = read_video_stream(config["dataset"]);
    vid->open();

    while (!vid->has_ended()) {
        cv::Mat frame = vid->get_next_frame();

        cv::imshow("detections", frame);
        cv::waitKey();
    }
}