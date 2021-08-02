#include <math.h>
#include <stdio.h>

#include <vector>


#include <opencv2/opencv.hpp>

#include "config.h"
#include "filter_corners.h"
#include "find_corners.h"
#include "get_init_location.h"
#include "image_normalization_and_gradients.h"
#include "non_maximum_suppression.h"
#include "polynomial_fit.h"
#include "refine_corners.h"
#include "score_corners.h"

namespace cbdetect {

void find_corners_reiszed(const cv::Mat& img, Corner& corners, const Params& params) {
  cv::Mat img_resized, img_norm;
  Corner corners_resized;

  // resize image
  double scale = 0;
  if(img.rows < 640 || img.cols < 480) {
    scale = 2.0;
  } else if(img.rows >= 640 || img.cols >= 480) {
    scale = 0.5;
  } else {
    return;
  }
  cv::resize(img, img_resized, cv::Size(img.cols * scale, img.rows * scale), 0, 0, cv::INTER_LINEAR);

  if(img_resized.channels() == 3) {
    cv::cvtColor(img_resized, img_norm, cv::COLOR_BGR2GRAY);
    img_norm.convertTo(img_norm, CV_64F, 1 / 255.0, 0);
  }
  else {
    img_resized.convertTo(img_norm, CV_64F, 1 / 255.0, 0);
  }

  // normalize image and calculate gradients
  cv::Mat img_du, img_dv, img_angle, img_weight;
  image_normalization_and_gradients(img_norm, img_du, img_dv, img_angle, img_weight, params);

  // get corner's initial locaiton
  get_init_location(img_norm, img_du, img_dv, corners_resized, params);
  if(corners_resized.p.empty()) {
    return;
  }
  // pre-filter corners according to zero crossings
  filter_corners(img_norm, img_angle, img_weight, corners_resized, params);
  // refinement
  refine_corners(img_du, img_dv, img_angle, img_weight, corners_resized, params);

  // merge corners
  std::for_each(corners_resized.p.begin(), corners_resized.p.end(), [&scale](auto& p) { p /= scale; });
  // std::for_each(corners_resized.r.begin(), corners_resized.r.end(), [&scale](auto &r) { r = (double) r / scale; });
  double min_dist_thr = scale > 1 ? 3 : 5;
  for(int i = 0; i < corners_resized.p.size(); ++i) {
    double min_dist = DBL_MAX;
    cv::Point2d& p2 = corners_resized.p[i];
    for(int j = 0; j < corners.p.size(); ++j) {
      cv::Point2d& p1 = corners.p[j];
      double dist     = cv::norm(p2 - p1);
      min_dist        = dist < min_dist ? dist : min_dist;
    }
    if(min_dist > min_dist_thr) {
      corners.p.emplace_back(corners_resized.p[i]);
      corners.r.emplace_back(corners_resized.r[i]);
      corners.v1.emplace_back(corners_resized.v1[i]);
      corners.v2.emplace_back(corners_resized.v2[i]);
      if(params.corner_type == MonkeySaddlePoint) {
        corners.v3.emplace_back(corners_resized.v3[i]);
      }
    }
  }
}

void find_corners(const cv::Mat& img, Corner& corners, const Params& params) {
  // clear old data
  corners.p.clear();
  corners.r.clear();
  corners.v1.clear();
  corners.v2.clear();
  corners.v3.clear();
  corners.score.clear();

  // convert to double grayscale image
  cv::Mat img_norm;
  if(img.channels() == 3) {
#if CV_VERSION_MAJOR >= 4
    cv::cvtColor(img, img_norm, cv::COLOR_BGR2GRAY);
#else
    cv::cvtColor(img, img_norm, CV_BGR2GRAY);
#endif
    img_norm.convertTo(img_norm, CV_64F, 1. / 255., 0);
  } else {
    img.convertTo(img_norm, CV_64F, 1. / 255., 0);
  }

  // normalize image and calculate gradients
  cv::Mat img_du, img_dv, img_angle, img_weight;
  image_normalization_and_gradients(img_norm, img_du, img_dv, img_angle, img_weight, params);

  // get corner's initial locaiton

  get_init_location(img_norm, img_du, img_dv, corners, params);
  if(corners.p.empty()) {
    return;
  }

  // pre-filter corners according to zero crossings
  filter_corners(img_norm, img_angle, img_weight, corners, params);

  // refinement
  refine_corners(img_du, img_dv, img_angle, img_weight, corners, params);


  // resize image to detect more corners
  find_corners_reiszed(img, corners, params);

  // polynomial fit
  if(params.polynomial_fit) {
    polynomial_fit(img_norm, corners, params);
  }

  // score corners
  sorce_corners(img_norm, img_weight, corners, params);

  // remove low scoring corners
  remove_low_scoring_corners(params.score_thr, corners, params);

  // non maximum suppression
  non_maximum_suppression_sparse(corners, 3, img.size(), params);
}

} // namespace cbdetect
