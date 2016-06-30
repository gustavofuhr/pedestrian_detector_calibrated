#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "json/json.h"

enum VideoStreamType {
	VIDEO_FILE = 0,
	IMAGE_SEQUENCE,
	IMAGE_DIRECTORY
};


class VideoStream {

	public:
		virtual	void open() = 0;
		virtual void close() = 0;
		virtual cv::Mat get_next_frame() = 0;
		virtual bool has_ended() = 0;
		virtual int get_current_frame_number() = 0;
		virtual VideoStreamType get_type() = 0;
};


VideoStream *read_video_stream(const std::string &config_filename);
VideoStream *read_video_stream(const Json::Value &config_node);
