#ifndef LIBCBDETECT_FIND_CORNERS_H
#define LIBCBDETECT_FIND_CORNERS_H

#include <vector>

#include <opencv2/opencv.hpp>

#include "config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL void find_corners(const cv::Mat& img, Corner& corners,
                                       const Params& params = Params());

}

#endif //CALIBRATION_FIND_CORNERS_H
