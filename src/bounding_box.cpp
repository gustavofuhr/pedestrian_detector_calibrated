#include "bounding_box.h"

void BoundingBox::plot (cv::Mat &frame, cv::Scalar color)
{
    cv::rectangle(frame, bb, color, 2.0);
}
