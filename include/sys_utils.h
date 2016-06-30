#pragma once

#include <set>
#include <vector>
#include <string>
#include <cstring>


std::vector<std::string> get_files_inside_directory(const std::string &image_dir, bool image_dir_recursive = false, std::set<std::string> filter_extensions = std::set<std::string>());
std::vector<std::string> get_dirs_inside_directory(const std::string &root_dir);
