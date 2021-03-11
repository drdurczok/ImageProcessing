#include "../inc/image_processing.hpp"

image_processing::image_processing(){}

Mat image_processing::filter(Mat input){
	if (input.empty()){
		cout << "Error: missing image\n\n" << endl;
	}
	//Convert image
	Mat output;
	cvtColor(input, output, COLOR_RGB2GRAY);

	//Make binary image
	threshold(output, output, 210, 255, THRESH_BINARY);

	GaussianBlur(output, output, 
		Size(5, 5), 					//Smoothing (width, height) in px
		1);								//Sigma, how much image blur

	return output;
}

Mat image_processing::find_edge(Mat input, edge_filter_methods selection){	
	Mat output;

	switch (selection) {
	    case SOBEL:
	      output = this->sobel_edge_detection(input);
	      break;
	    default:
	      output = this->sobel_edge_detection(input);
	}

	return output;
}

Mat image_processing::sobel_edge_detection(Mat input){
	Mat sobel, sobelx, sobely;

	/* 
	 * Sobel (input, output, ddepth, dx, dy, ksize=3, scale=1, delta=0, borderType=BORDER_DEFAULT)
	 *		ddepth - output image depth
	 *		dx, dy - order of the derivative
	 *		ksize  - size of the extended Sobel kernel; it must be 1, 3, 5, or 7
	 */

	Sobel(input, sobelx, CV_64F, 1, 0, 5);  // x
	Sobel(input, sobely, CV_64F, 0, 1, 5);  // y

	// Combine images together
	bitwise_and(sobelx, sobely, sobel);

	return sobel;
}

Mat image_processing::ellipse_detection(Mat input){


	return input;
}