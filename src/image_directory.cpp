#include "image_directory.h"
#include <fstream>

#include "sys_utils.h"

ImageDirectory::ImageDirectory(const std::string &config_filename) :
	image_dir_recursive(true),
	step_frame(1)
{

	// read the configuration file
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
				if (token == "image_dir" && s_line >> token && token == "=") 
					s_line >> image_dir;
				else if (token == "step_frame" && s_line >> token && token == "=") 
					s_line >> step_frame;
				else if (token == "image_dir_recursive" && s_line >> token && token == "=")  {
					std::string sbool;
					s_line >> sbool;
					image_dir_recursive = (sbool == "true");
				}
		
			}
		}
	}

	c_file.close();

	set_allowed_image_ext();
}

ImageDirectory::ImageDirectory(const Json::Value &config_node):
	image_dir_recursive(true),
	step_frame(1)
{
	image_dir = config_node["image_dir"].asString();
	step_frame = config_node.get("step_frame", 1).asInt();
	image_dir_recursive = config_node.get("image_dir_recursive", true).asBool();

	set_allowed_image_ext();
}

void ImageDirectory::set_allowed_image_ext() {
	// set the extension that will be searched in the directory
	allowed_image_extensions.insert(".jpeg");
	allowed_image_extensions.insert(".jp2");
	allowed_image_extensions.insert(".png");
	allowed_image_extensions.insert(".jpg");
	allowed_image_extensions.insert(".bmp");
    allowed_image_extensions.insert(".ppm");
}

void ImageDirectory::open() {
	current_frame = 0;
	image_files = get_files_inside_directory(image_dir, image_dir_recursive, allowed_image_extensions);
}

void ImageDirectory::close() {

}

bool ImageDirectory::has_ended() {
	return current_frame >= image_files.size();
}

cv::Mat ImageDirectory::get_next_frame() {
	cv::Mat image = cv::imread(image_files[current_frame]);
	current_frame += step_frame;

	return image;
}
