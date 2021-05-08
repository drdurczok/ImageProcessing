#ifndef IMAGE_CORE_H
#define IMAGE_CORE_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdint.h>

using namespace std;

class image_core {
  public:
  	image_core(uint8_t, uint8_t);
  	cv::Mat load_image(string);
  	void display_image(uint8_t, cv::Mat);
  	void save_image(string, cv::Mat);
	cv::Mat multiply_saturation_greyscale(cv::Mat, float);
	cv::Mat multiply_saturation_color(cv::Mat, float, int);
	cv::Mat get_DFT(cv::Mat, bool);

  private:
  	uint16_t screen_res_x = 240*2;
  	uint16_t screen_res_y = 320*2;

  	void prepare_windows(uint8_t, uint8_t);


};

#endif