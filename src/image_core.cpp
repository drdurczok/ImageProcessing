#include "../inc/image_core.hpp"

image_core::image_core(){}

cv::Mat image_core::load_image(std::string path_to_img){
	//Images are stored in Mat objects
	cv::Mat image;
	/*Options:
	 * cv::IMREAD_COLOR
	 * cv::IMREAD_GRAYSCALE
	 * cv::IMREAD_UNCHANGED
	 */
    image = cv::imread(path_to_img, cv::IMREAD_COLOR); // Read the file
    if( image.empty() ){
        std::cout << "Could not open or find the image" << std::endl;
    }
    return image;
}

void image_core::display_image(std::string Handle, cv::Mat image){
	/*Options:
	 * cv::WINDOW_NORMAL
	 * cv::WINDOW_AUTOSIZE		--User cannot adjust
	 * cv::WINDOW_FREERATIO		--User may resize window
	 * cv::WINDOW_FULLSCREEN
	 * cv::WINDOW_OPENGL
	 */
	cv::namedWindow(Handle, cv::WINDOW_FREERATIO);
    cv::imshow(Handle, image);
    cv::resizeWindow(Handle, image.cols/10, image.rows/10);
    cv::moveWindow(Handle, 1000, 300);
}

void image_core::save_image(std::string name, cv::Mat image){
	cv::imwrite(name, image);
}

cv::Mat image_core::multiply_saturation_greyscale(cv::Mat image, float mult){
	for(int r=0; r<image.rows; r++){
		for(int c=0; c<image.cols; c++){
			//GREYSCALE - uint8_t
			image.at<uint8_t>(r,c) = image.at<uint8_t>(r,c) * mult;
		}
	}
	return image;
}

cv::Mat image_core::multiply_saturation_color(cv::Mat image, float mult, int chan){
	// Channel: 0 - blue, 1 - green, 2 - red
	for(int r=0; r<image.rows; r++){
		for(int c=0; c<image.cols; c++){
			//COLOR     - cv::Vec3b
			image.at<cv::Vec3b>(r,c)[chan] = image.at<cv::Vec3b>(r,c)[chan] * mult;
		}
	}
	return image;
}