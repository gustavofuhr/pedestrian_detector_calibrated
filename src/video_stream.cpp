#include "video_stream.h"
#include <iostream>
#include <fstream>

#include "sys_utils.h"
#include "util.h"


#include "video_file.h"
#include "image_sequence.h"
#include "image_directory.h"

VideoStream *read_video_stream(const std::string &config_filename) {
	VideoStream *ret = NULL;

	std::ifstream c_file;

	std::string line;
	std::string token;
	
	c_file.open(config_filename.c_str());
	if (!c_file.is_open())
		std::cerr << "Could not open the file!" << std::endl;
	
	while (c_file.good()) {
		getline(c_file, line);

		if (line[0] != '%') {
			std::stringstream s_line(line);
			while (s_line >> token) {
				if (token == "image_pref") {
					c_file.close();
					ret = new ImageSequence(config_filename);	
					return ret;
				}
				else if (token == "video_file") {
					c_file.close();
					ret = new VideoFile(config_filename);	
					return ret;
				}
				else if (token == "image_dir") {
					c_file.close();
					ret = new ImageDirectory(config_filename);	
					return ret;	
				}
			}
		}
	}

	return ret;
}

VideoStream *read_video_stream(const Json::Value &config_node) {
	VideoStream *ret = NULL;

	if (!config_node["image_pref"].isNull()) {
		ret = new ImageSequence(config_node);	
		return ret;
	}
	else if (!config_node["video_file"].isNull()) {
		ret = new VideoFile(config_node);	
		return ret;
	}
	else if (!config_node["image_dir"].isNull()) {
		std::cout << "image_directory" << std::endl;
		ret = new ImageDirectory(config_node);	
		return ret;
	}
	else {
		std::cerr << "Not recognized type of stream!" << std::endl;
	}

	return ret;
}

