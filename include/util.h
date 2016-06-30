#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <time.h>

cv::Mat safe_open_img(const std::string &name, int flags=1);

std::string string_uppercase(const std::string &s);

bool string2bool(const std::string &s);

void print_dmatrix(const std::string &title, const cv::Mat &m);

//NOTE: assumes a one dimensional histogram
void normalize_hist(cv::Mat &hist);

cv::Mat_<double> angles2matrix(double phi, double theta, double psi);

// the im_point must be in homogeneous coordinates
// the matrix H is inverted in the function
cv::Mat_<double> image2world_H(cv::Mat &im_point, cv::Mat_<double> &H);
cv::Mat_<double> world2image_H(cv::Mat &im_point, cv::Mat_<double> &H);

cv::Mat sample_quadrilateral(cv::Mat &corners, double sample_step);

double bhattacharyya(std::vector<cv::Mat> &hist1, std::vector<cv::Mat> &hist2);

cv::Mat_<double> image2world(cv::Mat_<double> &im_point, cv::Mat_<double> &P, double world_height);
cv::Mat_<double> world2image(cv::Mat &w_point, cv::Mat_<double> &P);
double diffclock(clock_t clock1,clock_t clock2);

std::string format_int(int number, int mask_size);

std::vector<cv::Scalar> distinct_colors(int n_colors);

// write text in the image.
void matPrint(cv::Mat &img, int lineOffsY, cv::Scalar fontColor, const std::string &ss);
void matPrint_xy(cv::Mat &img, int bl_x, int bl_y, cv::Scalar font_color, const std::string &ss);

bool in_boundaries(cv::Point2i &p, int h, int w);

class mouse_handler {
	public:
		bool wait_click(cv::Point2i &ret_clicked_point);

		/*constructors*/
		mouse_handler(const std::string &window_name);

    private:
		bool clicked;
		cv::Point2i clicked_point;
		static void on_mouse( int event, int x, int y, int flags, void* param );
};
