#include "../inc/image_core.hpp"

image_core::image_core(){
	this->image_core(2,2);
}

image_core::image_core(uint8_t window_cols, uint8_t window_rows){
	cout << "Initiating image core with " 
		 << to_string(window_cols) << " by " 
		 << to_string(window_rows) << " windows." 
		 << endl;
	this->prepare_windows(window_cols, window_rows);
}

void image_core::prepare_windows(uint8_t window_cols, uint8_t window_rows){
	uint16_t y_size = (screen_res_y / window_cols);
	uint16_t x_size = (screen_res_x / window_rows);
	uint16_t y_pos;
	uint16_t x_pos;
	uint8_t handle_int = 0;
	string Handle;

	cout << "Image size: " << x_size << "x" << y_size << endl;

	for (uint8_t i = 0; i < window_cols; i++){
		for (uint8_t j = 0; j < window_rows; j++){
			Handle = to_string(handle_int++);
			
			/*Options:
			 * cv::WINDOW_NORMAL
			 * cv::WINDOW_AUTOSIZE		--User cannot adjust
			 * cv::WINDOW_FREERATIO		--User may resize window
			 * cv::WINDOW_FULLSCREEN
			 * cv::WINDOW_OPENGL
			 */
			cv::namedWindow(Handle, cv::WINDOW_FREERATIO);
		    //cv::resizeWindow(Handle, image.cols/10, image.rows/10);
		    cv::resizeWindow(Handle, x_size, y_size);

		    y_pos = 10 + (92*i) + y_size*i;
		    x_pos = 10 + (30*j) + x_size*j;
		    cv::moveWindow(Handle, x_pos, y_pos);

		    cout << "Position of window " << Handle 
		    	 << " is (X,Y) = (" << x_pos << "," << y_pos
		    	 << ")" << endl;
		}
	}
}

// Create Mat of zeroes using type CZ_8UC1
// Mat::zeros(channel.size(), CV_8UC1)

cv::Mat image_core::load_image(string path_to_img){
	//Images are stored in Mat objects
	cv::Mat image;
	/*Options:
	 * cv::IMREAD_COLOR
	 * cv::IMREAD_GRAYSCALE
	 * cv::IMREAD_UNCHANGED
	 */
    image = cv::imread(path_to_img, cv::IMREAD_COLOR); // Read the file
    if( image.empty() ){
        cout << "Could not open or find the image" << endl;
    }
    return image;
}

void image_core::display_image(uint8_t handle_int, cv::Mat image){
    cv::imshow(to_string(handle_int), image);
}

void image_core::save_image(string name, cv::Mat image){
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

cv::Mat image_core::get_DFT(cv::Mat image, bool visualize = 0){
	cv::Mat image_float;
	cv::Mat image_dft_ready;
	cv::Mat image_dft;

	image.convertTo(image_float, CV_32FC1, 1.0/255.0);

	// Two channel mat (real and imaginary)
	cv::Mat image_complex[2] = {image_float, cv::Mat::zeros(image.size(), CV_32F)};
	cv::merge(image_complex, 2, image_dft_ready);


	/*Options:
	 *	cv::DFT_COMPLEX_OUTPUT
	 *	cv::DFT_INVERSE
	 *	cv::DFT_REAL_OUTPUT
	 *	cv::DFT_ROWS
	 *	cv::DFT_SCALE
	 */
	cv::dft(image_dft_ready, image_dft, cv::DFT_COMPLEX_OUTPUT);

	if (visualize){

	}

	return image_dft;
}