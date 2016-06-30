#pragma once
#include "video_stream.h"
#include "json/json.h"


class VideoFile : public VideoStream {
	public:
		 void open();
		 void close();
		 bool has_ended();
		 cv::Mat get_next_frame();
		 void skip_n_frames(int n);
		 VideoFile(const std::string &config_filename);
		 VideoFile(const Json::Value &config_node);
		 inline int get_current_frame_number() {
		 	return current_frame;
		 };
		 inline VideoStreamType get_type() {
		 	return VIDEO_FILE;
		 };
	private:
		bool video_has_ended;
		std::string video_filename;
		int begin_frame, end_frame, step_frame, current_frame;
		cv::VideoCapture cap;
};
