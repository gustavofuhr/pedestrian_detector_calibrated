#include "sys_utils.h"
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <locale>


#include <iostream>
#include <algorithm>

int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct stat _buf;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {

        if(stat(dirp->d_name, &_buf) != 0x4)
        files.push_back(std::string(dirp->d_name));
    }

    // Make sure that the files are presented in alphabetic order
    sort( files.begin(), files.end() );

    closedir(dp);
    return 0;
}



std::vector<std::string> get_files_inside_directory(const std::string &image_dir, bool image_dir_recursive, std::set<std::string> filter_extensions) 
{
	DIR *dir = opendir(image_dir.c_str());
    if(dir == NULL) {
        std::cout << "Error(" << errno << ") opening " << image_dir << std::endl;
        return std::vector<std::string>();
    }
    std::locale loc;

	struct dirent *entry = readdir(dir);
	std::vector<std::string> res_files;

	while (entry != NULL) {
		if (entry->d_type == DT_DIR) {
			if (std::string(entry->d_name) != "..") {
				std::vector<std::string> files_in_dir;
				getdir(image_dir+"/"+entry->d_name, files_in_dir);
				for (int i = 0; i < files_in_dir.size(); ++i) {					
                    const char *ext = strrchr(files_in_dir[i].c_str(), '.');
                    if(ext == NULL)
                        continue;
                    std::string str_ext(ext);
                    for(int i=0; i<str_ext.size(); i++)
                        str_ext[i] = std::tolower(str_ext[i], loc);

                    if (filter_extensions.size() == 0 || (ext != NULL && filter_extensions.find(str_ext) != filter_extensions.end())) {
						if (std::string(entry->d_name) != ".")
							res_files.push_back(image_dir+"/"+entry->d_name+"/"+files_in_dir[i]);
						else
							res_files.push_back(image_dir+"/"+files_in_dir[i]);
					}
					
				}
			}
		}
		
		entry = readdir(dir);
	}

    closedir(dir);

	return res_files;
}

std::vector<std::string> get_dirs_inside_directory(const std::string &root_dir) 
{
    DIR *dir = opendir(root_dir.c_str());
    
    struct dirent *entry = readdir(dir);
    std::vector<std::string> res_dirs;

    while (entry != NULL) {
        if (entry->d_type == DT_DIR) {
            if (std::string(entry->d_name) != ".." && std::string(entry->d_name) != ".") {
                res_dirs.push_back(std::string(entry->d_name));
            }
        }
        entry = readdir(dir);
    }

    sort( res_dirs.begin(), res_dirs.end() );
    closedir(dir);

    return res_dirs;
}

