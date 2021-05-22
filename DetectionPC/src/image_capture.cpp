#include "../inc/image_capture.hpp"
#include "image_processing.cpp"

image_capture::image_capture(){

}

Mat image_capture::take_image(){
	//String path_to_image = "../samples/Opponent/1.jpg";
	String path_to_image = "../samples/1.jpg";

	Mat image = imread(path_to_image, IMREAD_COLOR);
    if( image.empty() ){
        cout << "Could not open or find the image" << endl;
    }

    return image;
}