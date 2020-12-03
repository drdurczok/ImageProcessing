#ifndef IMAGE_CORE_H
#define IMAGE_CORE_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdint.h>

class image_core {
  public:
  	image_core();
  	cv::Mat load_image(std::string);
  	void display_image(std::string Handle, cv::Mat);
  	void save_image(std::string, cv::Mat);
	cv::Mat multiply_saturation_greyscale(cv::Mat, float);
	cv::Mat multiply_saturation_color(cv::Mat, float, int);
};

#endif