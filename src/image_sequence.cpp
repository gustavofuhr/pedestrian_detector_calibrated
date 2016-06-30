#include "image_sequence.h"
#include <fstream>

#include "util.h"

ImageSequence::ImageSequence(const std::string &config_filename) {
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
				if (token == "image_pref" && s_line >> token && token == "=") 
					s_line >> image_pref;
				else if (token == "file_ext" && s_line >> token && token == "=") 
					s_line >> file_ext;
				else if (token == "begin_frame" && s_line >> token && token == "=") 
					s_line >> begin_frame;
				else if (token == "step_frame" && s_line >> token && token == "=") 
					s_line >> step_frame;
				else if (token == "end_frame" && s_line >> token && token == "=") 
					s_line >> end_frame;
				else if (token == "d_mask" && s_line >> token && token == "=") 
					s_line >> d_mask;
			}
		}
	}

	c_file.close();
}

ImageSequence::ImageSequence(const Json::Value &config_node)
{
	image_pref = config_node["image_pref"].asString();
	file_ext = config_node["file_ext"].asString();
	begin_frame = config_node["begin_frame"].asInt();
	step_frame = config_node["step_frame"].asInt();
	end_frame = config_node["end_frame"].asInt();
	d_mask = config_node["d_mask"].asInt();
}

void ImageSequence::open() {
	current_frame = begin_frame;
}

void ImageSequence::close() {
    current_frame = end_frame;
}

bool ImageSequence::has_ended() {
	return (current_frame >= end_frame);
} 

cv::Mat ImageSequence::get_next_frame() {
	std::stringstream ss;
	ss << image_pref << format_int(current_frame, d_mask) << "." << file_ext;

    // TODO: read in gray scale or what?
    cv::Mat im = cv::imread(ss.str(), 0);

    current_frame += step_frame;

    return im;
}