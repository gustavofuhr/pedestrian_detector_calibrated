#include "video_file.h"
#include <fstream>

VideoFile::VideoFile(const std::string &config_filename) :
	begin_frame(-1),
	end_frame(-1),
	step_frame(1),
	current_frame(0),
	video_has_ended(false)
{
	std::ifstream c_file;

	std::string line;
	std::string token;
	
	c_file.open(config_filename.c_str());
	if (!c_file.is_open())
        std::cerr << "VideoFile::Could not open the file!" << std::endl;
	
	while (c_file.good()) {
		getline(c_file, line);

		if (line[0] != '%') {
			std::stringstream s_line(line);
			while (s_line >> token) {			
				if (token == "video_file" && s_line >> token && token == "=") 
					s_line >> video_filename;
				else if (token == "begin_frame" && s_line >> token && token == "=") 
					s_line >> begin_frame;
				else if (token == "step_frame" && s_line >> token && token == "=") 
					s_line >> step_frame;
				else if (token == "end_frame" && s_line >> token && token == "=") 
					s_line >> end_frame;
			}
		}
	}
	
	c_file.close();
}

VideoFile::VideoFile(const Json::Value &config_node):
	begin_frame(-1),
	end_frame(-1),
	step_frame(1),
	current_frame(0),
	video_has_ended(false)
{
	video_filename = config_node["video_file"].asString();
	begin_frame = config_node.get("begin_frame", -1).asInt();
	step_frame = config_node.get("step_frame", 1).asInt();
	end_frame = config_node.get("end_frame", -1).asInt();
}

void VideoFile::open() {
	cap.open(video_filename); 

	if(!cap.isOpened()) 
		std::cerr << "Not able to open video file: " << video_filename << std::endl;
	else {
		// skip frames if begin_frame is not 1
		if (begin_frame > 1)
			skip_n_frames(begin_frame-1);
	}

}

void VideoFile::close() {
    cap.release();
}

void VideoFile::skip_n_frames(int n) {
	for (int i = 0; i < n; ++i) {
		cap.grab();
		current_frame++;
	}
}

bool VideoFile::has_ended() {
	return (video_has_ended || (current_frame >= end_frame && end_frame != 0));
} 


cv::Mat VideoFile::get_next_frame() {
	// skip if step frame is larger than one
	if (step_frame > 1 && current_frame > 0) {
		skip_n_frames(step_frame-1);
	}
	cv::Mat frame;	
	cap >> frame;
	if (frame.empty()) 
		video_has_ended = true;
	current_frame++;
    return frame;
}
