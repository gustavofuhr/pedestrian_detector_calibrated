#pragma once
#include "video_stream.h"
#include "json/json.h"

class ImageSequence : public VideoStream {
	public:
		void open();
        void close();
        bool has_ended();
		cv::Mat get_next_frame();
		ImageSequence(const std::string &config_filename);
		ImageSequence(const Json::Value &config_node);
		inline int get_current_frame_number() {
		 	return current_frame;
		 };
		 inline VideoStreamType get_type() {
		 	return IMAGE_SEQUENCE;
		 };
	private:
		std::string image_pref, file_ext;
		int begin_frame, end_frame, current_frame, step_frame;
		int d_mask;

};