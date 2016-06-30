#pragma once
#include <set>
#include "video_stream.h"
#include "json/json.h"

class ImageDirectory : public VideoStream {
	public:
		void open();
        void close();
        bool has_ended();
		cv::Mat get_next_frame();
		ImageDirectory(const std::string &config_filename);
		ImageDirectory(const Json::Value &config_node);
		inline int get_current_frame_number() {
		 	return current_frame;
		};
		inline VideoStreamType get_type() {
		 	return IMAGE_DIRECTORY;
		};

	private:
		void set_allowed_image_ext();
		std::vector<std::string> image_files;
		std::set<std::string> allowed_image_extensions;
		std::string image_dir;
		int step_frame, current_frame;
		bool image_dir_recursive;
};