#pragma once

#include "video_stream.h"
#include "json/json.h"

class PedestrianDetector {

public:
    PedestrianDetector(const std::string &config_filename);
    void runDetection();


private:
    Json::Value config;
};