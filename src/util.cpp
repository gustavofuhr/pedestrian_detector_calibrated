
#include "util.h"
// #include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui.hpp>


cv::Mat safe_open_img(const std::string& name, int flags) {
    cv::Mat img = cv::imread(name, flags);

    if(img.rows <= 0)
        std::cerr << "Problem opening image: |" << name << "|" << std::endl;

    return img;
}

int upper(int c) {
	return std::toupper((unsigned char)c);
}


std::string string_uppercase(const std::string &s) {

	std::string s_out(s);
	std::transform(s_out.begin(), s_out.end(), s_out.begin(), upper);

	return s_out;

}

bool string2bool(const std::string &s) {

	if (string_uppercase(s) == "TRUE")
		return true;
	else if (string_uppercase(s) == "FALSE")
		return false;
	else std::cerr << "The given string does not contain a boolean value!" << std::endl;

    return false;
}


void print_dmatrix(const std::string &title, const cv::Mat &m) {

	std::cout << title << std::endl;
	for (int i = 0; i < m.rows; ++i) {
		for (int j = 0; j < m.cols; ++j) {
			std::cout << m.at<double>(i,j) << " ";
		}
		std::cout << std::endl;
	}
}


cv::Mat_<double> angles2matrix(double phi, double theta, double psi) {

	double r_x[9] = {1.0,      0.0,       0.0, 
				     0.0, cos(phi), -sin(phi),
					 0.0, sin(phi),  cos(phi)};
	cv::Mat_<double> Rx(3, 3, r_x);

	double r_y[9] = {cos(theta),    0.0,  sin(theta), 
					        0.0,    1.0,         0.0,
				   	-sin(theta),    0.0,  cos(theta)};
	cv::Mat_<double> Ry(3, 3, r_y);

	double r_z[9] = {cos(psi),  -sin(psi),     0.0, 
					 sin(psi),   cos(psi),     0.0,
					   	  0.0,        0.0,     1.0};
	cv::Mat_<double> Rz(3, 3, r_z);

	cv::Mat_<double> result = Rz * Ry * Rx;
	
	return result;
}


cv::Mat_<double> image2world_H(cv::Mat &im_point, cv::Mat_<double> &H) {
	
	cv::Mat wp;
	wp  	 = H.inv()*im_point;

	for (int i = 0; i < im_point.cols; i++) {
		// normalize the point
		wp.at<double>(0, i) = wp.at<double>(0, i)/wp.at<double>(2, i);
		wp.at<double>(1, i) = wp.at<double>(1, i)/wp.at<double>(2, i);
	}

	cv::Mat res;
	res = wp(cv::Range(0,2), cv::Range::all());
	return res;
}

cv::Mat_<double> world2image_H(cv::Mat &im_point, cv::Mat_<double> &H) {
	
	cv::Mat imp;
	imp   = H*im_point;

	for (int i = 0; i < im_point.cols; i++) {
		// normalize the point
		imp.at<double>(0,i) = imp.at<double>(0,i)/imp.at<double>(2,i);
		imp.at<double>(1,i) = imp.at<double>(1,i)/imp.at<double>(2,i);
	}

	cv::Mat res;
	res = imp(cv::Range(0,2), cv::Range::all());
	return res;
}

cv::Mat_<double> world2image(cv::Mat &w_point, cv::Mat_<double> &P) {
	
	cv::Mat wp;
	wp   = P*w_point;

	for (int i = 0; i < w_point.cols; i++) {
		// normalize the point
		wp.at<double>(0,i) = wp.at<double>(0,i)/wp.at<double>(2,i);
		wp.at<double>(1,i) = wp.at<double>(1,i)/wp.at<double>(2,i);
	}

	cv::Mat res = wp(cv::Range(0,2), cv::Range::all());
	return res;
}

void normalize_hist(cv::Mat &hist) {

	double s = cv::sum(hist)[0];
	hist = hist/s;		
}

cv::Mat sample_quadrilateral(cv::Mat &corners, double sample_step) {
	cv::Mat p1, p2, p3, p4;
	p1 = corners.col(0); // first corner
	p2 = corners.col(1); 
	p3 = corners.col(2); 
	p4 = corners.col(3); // last corner

	int n_spoints = floor(cv::norm(p1 - p4)/sample_step);
	
	cv::Mat_<double> pts_l(2, n_spoints), pts_r(2, n_spoints);

	// sample one side of the quadrilateral
	// and the opposite side, using the same n of points
	double step_t = 1.0/(n_spoints-1);
	double t = 0;
	for (int i = 0; i < n_spoints; ++i) {
		pts_l.col(i) = t*p2 + (1-t)*p3;
		pts_r.col(i) = t*p1 + (1-t)*p4;

		t += step_t;
	}
	
	// now sample the horizontal lines formed by the pts_l and pts_r
	int n_hpoints = floor(cv::norm(p3 - p4)/sample_step);

	cv::Mat_<double> pts(2, n_hpoints*n_spoints);
	step_t = 1.0/(n_hpoints - 1);
	int i_p = 0;
	for (int i = 0; i < n_spoints; ++i) {
		t = 0;
		for (int j = 0; j < n_hpoints; ++j) {
			//int i_p = (i)*(n_spoints-1) + j;
			double pts_x, pts_y;
			pts_x = t*pts_l(0,i) + (1 - t)*pts_r(0,i);
			pts_y = t*pts_l(1,i) + (1 - t)*pts_r(1,i);
			
			pts(0, i_p) = round(pts_x);
			pts(1, i_p) = round(pts_y);
			t += step_t;
			i_p++;
		}
	}

	return pts;
}


double bhattacharyya(std::vector<cv::Mat> &hist1, std::vector<cv::Mat> &hist2) {
	// this function only works if the histograms are float
	assert(hist1[0].type() == CV_32F && hist2[0].type() == CV_32F); 

	int n_chs = hist1.size(); // number of independent channels
	assert(n_chs == hist2.size());
	
	int n_bins = hist1[0].rows; // number of bins
	assert(n_bins == hist2[0].rows);

	double dist = 0;
	for (int c = 0; c < n_chs; ++c) {
		dist = dist + cv::compareHist(hist1[c], hist2[c], CV_COMP_BHATTACHARYYA);
	}
	return dist/n_chs;
}


/* BEFORE using compareHist
double bhattacharyya(std::vector<cv::Mat> &hist1, std::vector<cv::Mat> &hist2) {
	// this function only works if the histograms are float
	assert(hist1[0].type() == CV_32F && hist2[0].type() == CV_32F); 

	int n_chs = hist1.size(); // number of independent channels
	assert(n_chs == hist2.size());
	
	int n_bins = hist1[0].rows; // number of bins
	assert(n_bins == hist2[0].rows);

	double dist = 0;
	for (int c = 0; c < n_chs; ++c) {
		double bc = 0;// bhattacharrya coefficient

		for (int j = 0; j < n_bins; ++j) 
			bc = bc + sqrt(hist1[c].at<float>(j) * hist2[c].at<float>(j));

		dist = dist + sqrt(1 - bc);
	}
	return dist/n_chs;
}*/


/*cv::Mat_<double> image2world(cv::Mat_<double> &im_point, cv::Mat_<double> &P, double world_height) {

	double u = im_point(0);
	double v = im_point(1);

	// the next few lines solve a 2 by 2 linear system
	cv::Mat_<double> A(2, 2, 0.0);
	cv::Mat_<double> b(2, 1, 0.0);

	A(0,0) = P(2,0)*u - P(0,0);
	A(0,1) = P(2,1)*u - P(0,1);
	A(1,0) = P(2,0)*v - P(1,0);
	A(1,1) = P(2,1)*v - P(1,1);
	b(0)   = P(0,2)*world_height + P(0,3) - P(2,2)*world_height*u - P(2,3)*u;
	b(1)   = P(1,2)*world_height + P(1,3) - P(2,2)*world_height*v - P(2,3)*v;

	cv::Mat_<double> res(2, 1, 0.0);
	res = A.inv()*b;

	return res;
}*/




/* old version, without using matrix inversion */
cv::Mat_<double> image2world(cv::Mat_<double> &im_point, cv::Mat_<double> &P, double world_height) {


	double u = im_point(0);
	double v = im_point(1);

	// the next few lines solve a 2 by 2 linear system
	double a11, a12, a21, a22, b1, b2;


	a11 = P(2,0)*u - P(0,0);
	a12 = P(2,1)*u - P(0,1);
	a21 = P(2,0)*v - P(1,0);
	a22 = P(2,1)*v - P(1,1);
	b1  = P(0,2)*world_height + P(0,3) - P(2,2)*world_height*u - P(2,3)*u;
	b2  = P(1,2)*world_height + P(1,3) - P(2,2)*world_height*v - P(2,3)*v;


	cv::Mat_<double> res(2, 1, 0.0);
	res(1) = b2 - (a21 * b1)/a11;
	res(1) = res(1)/(a22 - (a21*a12)/a11);
	res(0) = (b1 - a12*res(1))/a11;


	return res;
}



double diffclock(clock_t clock1,clock_t clock2)
{
	double diffticks=clock1-clock2;
	double diffms=(diffticks*1000)/CLOCKS_PER_SEC;
	return diffms;
} 


std::string format_int(int number, int mask_size) {
	char smask[5], snum[10];
	
	sprintf(smask, "%%.%dd", mask_size);
	sprintf(snum, smask, number);

	return std::string(snum);
}



std::vector<cv::Scalar> distinct_colors(int n_colors) {
	// using the HSV colormap creates n equidistant points

	cv::Mat in_values(n_colors, 1, CV_8UC1);	

	double step  = 1.0/(n_colors-1);
	double value = 0.0;

	in_values.at<uchar>(0) = 0;
	for (int i = 1; i < n_colors; ++i) {
		value += step;
		in_values.at<uchar>(i) = (int)(value*200);
	}

	// applies the colomap
	cv::Mat out_values;
	applyColorMap(in_values, out_values, cv::COLORMAP_HSV);
	
	std::vector<cv::Scalar> colors;
	for (int i = 0; i < out_values.rows; ++i) {
		colors.push_back(cv::Scalar(out_values.at<uchar>(i,0), out_values.at<uchar>(i,1),
								out_values.at<uchar>(i,2) ));
	}

	return colors;
}


void mouse_handler::on_mouse( int event, int x, int y, int flags, void* param ){

	if (event == CV_EVENT_LBUTTONDOWN) {
		mouse_handler *mh;
		mh = static_cast<mouse_handler*>(param);

		mh->clicked = true;
		mh->clicked_point.x = x;
		mh->clicked_point.y = y;
	}
}

mouse_handler::mouse_handler(const std::string &window_name) : clicked(false) {

	if (cvGetWindowHandle(window_name.c_str()) == NULL) {
		std::cerr << "First you need to create the window." << std::endl;
		exit(1);
	}

	cv::setMouseCallback(window_name.c_str(), on_mouse, this);
}

bool mouse_handler::wait_click(cv::Point2i &ret_clicked_point) {
	clicked = false;
	int key = 0;

	while (!clicked) {
		key = cv::waitKey(20);      
		if ((char)key == 27) break;
	}

	if (clicked)
		ret_clicked_point = clicked_point;

	return clicked;
}

void matPrint(cv::Mat &img, int lineOffsY, cv::Scalar fontColor, const std::string &ss) {
	int fontFace = cv::FONT_HERSHEY_DUPLEX;
	double fontScale = 0.8;
	int fontThickness = 2;
	cv::Size fontSize = cv::getTextSize("T[]", fontFace, fontScale, fontThickness, 0);

	cv::Point org;
	org.x = 1;
	org.y = 3 * fontSize.height * (lineOffsY + 1) / 2;
	cv::putText(img, ss, org, fontFace, fontScale, CV_RGB(0,0,0), 5*fontThickness/2, 16);
	cv::putText(img, ss, org, fontFace, fontScale, fontColor, fontThickness, 16);
}



void matPrint_xy(cv::Mat &img, int bl_x, int bl_y, cv::Scalar font_color, const std::string &ss) {
	int fontFace = cv::FONT_HERSHEY_DUPLEX;
	double fontScale = 0.8;
	int fontThickness = 2;
	cv::Size fontSize = cv::getTextSize("T[]", fontFace, fontScale, fontThickness, 0);

	cv::Point org;
	org.x = bl_x;
	org.y = bl_y + fontSize.height;
	cv::putText(img, ss, org, fontFace, fontScale, CV_RGB(0,0,0), 5*fontThickness/2, 16);
	cv::putText(img, ss, org, fontFace, fontScale, font_color, fontThickness, 16);
}

bool in_boundaries(cv::Point2i &p, int h, int w) {
	if (p.x < w && p.x >= 0 && p.y < h && p.y >= 0)
		return true;
	else return false;
}
