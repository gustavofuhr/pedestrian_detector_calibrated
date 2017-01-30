#include "pedestrian_detector.h"
#include <fstream>
#include <cv.h>
#include <highgui.h>
#include <sstream>
#include "util.h"

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
                                        const std::vector<double> weights /* =std::vector<double>()*/) {
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

    // std::cout << "P " << P << std::endl;
}



void PedestrianDetector::grow_candidate(BoundingBox &candidate, float factor) {
    // this function is use to grow the bounding box to adapt to HoG model whose
    // training was done with some padding around the person.
    float new_width = candidate.bb.width*(1+factor);
    float new_height = candidate.bb.height*(1+factor);
    float new_x = candidate.bb.x - (new_width - candidate.bb.width)/2.0;
    float new_y = candidate.bb.y - (new_height - candidate.bb.height)/2.0;

    candidate.bb.width = new_width;
    candidate.bb.height = new_height;
    candidate.bb.x = new_x;
    candidate.bb.y = new_y;

}


std::vector<BoundingBox> PedestrianDetector::generateCandidatesWCalibration(int imageHeight, int imageWidth, double *maxHeight,
                            float meanHeight/* = 1.8m*/, float stdHeight/* = 0.1m*/, float factorStdHeight/* = 2.0*/) 
{

    // there is a set of parameters here that are hard coded, but should
    // be read from a file or something...
    cv::Mat_<float> P3 = P.col(2);

    //std::cout << "P3: " << P3 << std::endl;
    float aspectRatio = 0.5;
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

    float padding = 10;

    // create foot points using the pixels of the image
    for (int u = 0; u < imageWidth; u+=padding) {
        for (int v = minImageHeight; v < imageHeight; v+=padding ) {

            float Xw = (H_inv(0,0)*u + H_inv(0,1)*v + H_inv(0,2))/(H_inv(2,0)*u + H_inv(2,1)*v + H_inv(2,2));
            float Yw = (H_inv(1,0)*u + H_inv(1,1)*v + H_inv(1,2))/(H_inv(2,0)*u + H_inv(2,1)*v + H_inv(2,2));

            // now create all_candidates at different heights
            for (float h = -stdHeight * factorStdHeight; h <= stdHeight * factorStdHeight; h+= stepHeight) {
                // std::cout << h << std::endl;
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

                    grow_candidate(candidate, 0.2);
                    if (candidate.bb.x >= 0 && candidate.bb.x+candidate.bb.width < imageWidth && 
                            candidate.bb.y >= 0 && candidate.bb.y+candidate.bb.height < imageHeight &&
                            candidate.bb.height >= minImageHeight) {
                        if (candidate.bb.height > max_h) 
                            max_h = candidate.bb.height;
                        all_candidates.push_back(candidate);
                    }
                }

            }

            // exit(10);
            
        }
    }
    
    if (maxHeight != NULL) {
        *maxHeight = max_h;    
    }

    return all_candidates;
}


/*
    This function return the scale *level* in which the pyramid will be more fitted
*/
int PedestrianDetector::findClosestScaleFromBbox(BoundingBox &bb, int origin_image_height, const std::vector<float> &pyramid_scales) 
{
    // actually here is the size of the the image that changes, the model stays the same
    // to see the best fit for the bounding box, one must find the relation between the original
    // and then find the closest to the size of the bounding box
    int modelHeight = det_size.height;
    float min_dist = origin_image_height;
    int i_min = -1;

    for (int i = 0; i < pyramid_scales.size(); i++) {
        float scale = pyramid_scales[i];
        float diff = fabs(det_size.height/scale - bb.bb.height);

        if (diff < min_dist) {
            i_min = i;
            min_dist = diff;
        }
        else break;

    }

    return i_min;
}


std::vector<BoundingBox> PedestrianDetector::nonMaxSuppression(const std::vector<BoundingBox> &detections, 
                                                                const double final_threshold) {
    // create a vector of rectangles and weights
    std::vector<cv::Rect> rects;
    std::vector<double> ws;
    for (int i = 0; i < detections.size(); ++i) {
        // hack: I duplicate the rects such that they are all grouped with somebody.
        rects.push_back(detections[i].bb);
        ws.push_back(detections[i].score);
        rects.push_back(detections[i].bb);
        ws.push_back(detections[i].score);
    }

    hog.groupRectangles(rects, ws, final_threshold, 0.1);

    // for the data that was left, create bounding boxes
    std::vector<BoundingBox> new_ones;
    for (int i = 0; i < rects.size(); ++i) {
        BoundingBox det;
        det.bb = rects[i];
        det.score = ws[i];
        new_ones.push_back(det);
    }

    return new_ones;
}


/*
This function transforms the bounding boxes in pairs of scales and top-left points
*/
// std::vector<cv::Point> PedestrianDetector::computeSearchLocations(int imageHeight, int imageWidth, std::vector<cv::Rect> &bbox_candidates, float scale_pyramid) {

// }

std::vector<cv::Mat> PedestrianDetector::computeImagePyramid(cv::Mat &image, std::vector<float> &pyramid_scales, float scale_parameter, int n_levels) {

    // this only downsamples to create a pyramid and set the scale values
    float current_scale = 1.0;
    
    cv::Size curr_size = image.size();

    std::vector<cv::Mat> pyramid;
    pyramid.push_back(image);
    pyramid_scales.push_back(1.0);

    for (int i = 0; i < n_levels; ++i) {
        curr_size.width = curr_size.width/scale_parameter;
        curr_size.height = curr_size.height/scale_parameter;

        // std::cout << "scale_parameter " << scale_parameter << std::endl;
        // std::cout << "curr_size " << curr_size << std::endl;
        
        current_scale *= scale_parameter;

        // std::cout << "Size now " << curr_size << std::endl;

        cv::Mat im_new;
        cv::resize(pyramid[pyramid.size()-1], im_new, curr_size);
        pyramid.push_back(im_new);

        pyramid_scales.push_back(1.0/current_scale);
    }

    // std::cout << "pyramid inside: " << pyramid.size() << std::endl;
    return pyramid;
}


void PedestrianDetector::associateScaleToCandidates(std::vector<BoundingBox> &candidates, 
                                                        const std::vector<float> &pyramid_scales,
                                                        int original_image_height) {

    for (int i=0; i < candidates.size(); ++i) {
        int i_level = findClosestScaleFromBbox(candidates[i], original_image_height, pyramid_scales);
        candidates[i].pyramid_level = i_level;
    }

}


void debugCandidates(cv::Mat &frame, std::vector<BoundingBox> &candidates) {
    // std::cout << "number of candidates" << candidates.size() << std::endl;
    for (int i = 0; i < candidates.size(); ++i) {
        // std::cout << "c " << candidates[i].bb << std::endl;
        candidates[i].plot(frame, cv::Scalar(255,0,0));
    }
    cv::imshow("debug", frame);
    cv::waitKey(0);
}

double gaussianFunction(double mean, double std, double x) {
    return exp(-pow(x-mean, 2)/(2*pow(std,2)));
}

std::vector<BoundingBox> PedestrianDetector::detectWCandidates(std::vector<BoundingBox> &candidates, 
                                        const std::vector<cv::Mat> &pyramid_images,
                                        const std::vector<float> &pyramid_scales,
                                        const float hit_threshold) {

    std::vector<BoundingBox> detections;

    // for all the levels of the pyramid ...
    for (int l=0; l<pyramid_images.size(); ++l) {
        // ... search for candidates at this level and put them on search locations.
        std::cout << std::endl << "LEVEL: " << l << std::endl;
        std::cout << std::endl << "pyramid_scales[l]: " << pyramid_scales[l] << std::endl;
        // TODO: obviously I could order them by scale and reduce the complexity of this.
        std::vector<cv::Point> search_locations;
        for (int i=0; i<candidates.size(); ++i) { 
            
            if (l == candidates[i].pyramid_level) {
                // see if the bounding box of at this level will correspond to a
                // good approximation of the bounding box in the candidate
                float det_size_scale = det_size.height/pyramid_scales[l];
                // std::cout << "level in pyramid " << l << std::endl;
                // std::cout << "det at this scale representation height: " << det_size_scale << std::endl;
                // std::cout << "bb height " << candidates[i].bb.height << std::endl;

                // add candidate to search_location (top-left point)
                cv::Point pt = candidates[i].bb.tl()*pyramid_scales[l];
                search_locations.push_back(pt);
            }
        }

        std::cout << "search_locations size" << search_locations.size() << std::endl;

        // detect in this level.
        std::vector<cv::Point> found;
        std::vector<double> weights;
        hog.detect(pyramid_images[l], found, weights, hit_threshold, cv::Size(), cv::Size(), search_locations);

        // std::cout << std::endl << "Search: ";
        // for (int i=0; i<search_locations.size(); ++i) {
        //     std::cout << search_locations[i] << " ";
        // }
        // std::cout << std::endl << "Found: ";
        cv::Size det_size_l(det_size); // size of detector in this level
        det_size_l.width /= pyramid_scales[l];
        det_size_l.height /= pyramid_scales[l];
        for (int i = 0; i < found.size(); ++i) {
            BoundingBox bb;

            // find candidate to compute the score using gaussian weighthing
            for (int j=0; j<candidates.size(); ++j) {
                if (found[i] == candidates[j].bb.tl()*pyramid_scales[l]) {
                    // add weight for non max suppression.
                    double new_score = weights[i]*gaussianFunction(1.800, 0.300, candidates[j].world_height);
                    bb.score = new_score;
                    //std::cout << "det " << i << "score "<< new_score << std::endl;
                }
            }

            // bb.score = weights[i];
            cv::Point f = found[i];
            f.x /= pyramid_scales[l];
            f.y /= pyramid_scales[l];
            bb.bb = cv::Rect(f, det_size_l);

            detections.push_back(bb);

        }
    }

    return detections;
}

std::vector<BoundingBox> PedestrianDetector::detectBaseline(const std::vector<cv::Mat> pyramid_images, 
                                        const std::vector<float> pyramid_scales,
                                        const float hit_threshold,
                                        const float padding /* = 10*/)
{
    std::vector<BoundingBox> detections;
    int n_candidates = 0;
    
    for (int l=0; l<pyramid_images.size(); ++l) {
        // search locations are all points with the detector can handle
        std::vector<cv::Point> search_locations;
        for (int x = 0; x < pyramid_images[l].cols; x+=padding) {
            for (int y = 0; y < pyramid_images[l].rows; y+=padding) {
                search_locations.push_back(cv::Point(x,y));
                n_candidates++;
            }
        }

        // now use the search locations (sliding-window) to detect
        std::vector<cv::Point> found;
        std::vector<double> weights;
        hog.detect(pyramid_images[l], found, weights, hit_threshold, cv::Size(), cv::Size(), search_locations);

        // create bounding boxes with the found locations
        cv::Size det_size_l(det_size); // size of detector in this level
        det_size_l.width /= pyramid_scales[l];
        det_size_l.height /= pyramid_scales[l];
        for (int i = 0; i < found.size(); ++i) {
            BoundingBox bb;
            bb.score = weights[i];
            cv::Point f = found[i];
            f.x /= pyramid_scales[l];
            f.y /= pyramid_scales[l];
            bb.bb = cv::Rect(f, det_size_l);

            detections.push_back(bb);

        }
    }

    std::cout << "baseline number of candidates " << n_candidates << std::endl;
 
    return detections;
}

void PedestrianDetector::showDetections(cv::Mat &image, 
                                        std::vector<BoundingBox> &detections,
                                        cv::Scalar color) {
    for (int i=0; i<detections.size(); ++i) {
        detections[i].plot(image, color);
    }
    cv::imshow("Detections", image);
    cv::waitKey(1.0);
}



void PedestrianDetector::runDetection() {
    // open the video stream
    VideoStream *vid = read_video_stream(config["dataset"]);
    vid->open();

    std::vector<BoundingBox> candidates;
    double hit_threshold = config["detector_opts"]["hit_threshold"].asDouble();

    std::ofstream out_file;
    if (config["output"]["save_log"].asBool()) {
        //std::cout << "It's going to save a log file in " << config.logFilename << std::endl;
        out_file.open(config["output"]["out_filename"].asString());
        if (out_file.fail()) {
            std::cerr << "open failure: " << strerror(errno) << '\n';
        }
    }

    while (!vid->has_ended()) {
        cv::Mat frame = vid->get_next_frame();

        // resize frame if needed.
        if (config["detector_opts"]["resize_image"].asDouble() != 1.0) {
            double f = config["detector_opts"]["resize_image"].asDouble();
            cv::resize(frame, frame, cv::Size(), f, f);
        }

        // begin timer
        clock_t frame_start = clock();

        std::vector<BoundingBox> detections;
        // if calibration is required and the candidates were not build yet
        if (config["detector_opts"]["use_calibration"].asBool()) {
            if (candidates.size() == 0) {
                candidates = generateCandidatesWCalibration(frame.rows, frame.cols, NULL);
                std::cout << "Number of candidates: " << candidates.size() << std::endl; 
            }

            // std::cout << "Its going to debug..." << std::endl;
            // debugCandidates(frame, candidates);
            std::vector<cv::Mat> pyramid_images;
            std::vector<float> pyramid_scales;
            pyramid_images = computeImagePyramid(frame, pyramid_scales, 1.05);

            associateScaleToCandidates(candidates, pyramid_scales, frame.rows);
            detections = detectWCandidates(candidates, pyramid_images, pyramid_scales, hit_threshold);

            std::cout << "Number of detections: " << detections.size() << std::endl;
            
        }
        else {
            std::vector<cv::Mat> pyramid_images;
            std::vector<float> pyramid_scales;
            pyramid_images = computeImagePyramid(frame, pyramid_scales, 1.05);

            detections = detectBaseline(pyramid_images, pyramid_scales, hit_threshold);
        }

        showDetections(frame, detections, cv::Scalar(0,200,0));
        detections = nonMaxSuppression(detections);
        showDetections(frame, detections, cv::Scalar(0,0,200));

        clock_t frame_end = clock();
        std::cout << "TIME: " << (double(frame_end - frame_start) / CLOCKS_PER_SEC) << std::endl;
        // end timer
        
        int f = vid->get_current_frame_number();
        if (config["output"]["save_frames"].asBool()) {
            std::stringstream ss;
            ss << config["output"]["out_folder"].asString() << format_int(f, 4) << ".jpg";
            std::cout << "Saving frame " << ss.str() << std::endl;
            cv::imwrite(ss.str(), frame);

        }

        if (config["output"]["save_log"].asBool()) {
            for (int i = 0; i < detections.size(); ++i) {
                out_file << f << " " << detections[i].bb.x << " " 
                         << detections[i].bb.y << " "  
                         << detections[i].bb.height << " "
                         << detections[i].bb.width << " "
                         << detections[i].score << std::endl;
            }
        }
    }
}


// code to debug pyramid
// std::cout << "pyramid_images: " << pyramid_images.size();

// for (int i=0; i<pyramid_images.size(); ++i) {
//     std::cout << "pyramid level " << i << std::endl;
//     std::cout << "image size " << pyramid_images[i].size() << std::endl;
//     std::cout << "scale factor " << pyramid_scales[i] << std::endl;
//     cv::imshow("pyr", pyramid_images[i]);
//     cv::waitKey();
//     std::cout << "--" << std::endl;
// }
