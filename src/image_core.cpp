#include "../inc/image_core.hpp"

image_core::image_core(){}

cv::Mat image_core::get_image(){
	cv::Mat image;
    image = imread("../samples/images/IMG_001.jpg", cv::IMREAD_COLOR); // Read the file
    if( image.empty() ){
        std::cout << "Could not open or find the image" << std::endl;
    }
    return image;
}