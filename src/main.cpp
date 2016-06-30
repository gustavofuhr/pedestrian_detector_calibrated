
#include <iostream>
#include "pedestrian_detector.h"

int main(int argc, char* argv[]) {

    if (argc != 2) {
        std::cout << "Usage: bin/pedestrian_detector config.json" << std::endl;
        exit(1);
    }

    PedestrianDetector detector(argv[1]);
    detector.runDetection();

    return 0;
}