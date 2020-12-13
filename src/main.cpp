#include <opencv2/highgui.hpp>
#include <iostream>

#include "image_core.cpp"

using namespace cv;
using namespace std;

int main(){
	string path_to_image = "../samples/images/IMG_001.jpg";
	image_core img_core(2,2);
	
	Mat image = img_core.load_image(path_to_image);
	img_core.display_image(0, image);
	//img_core.save_image("temp.jpg",image);

	cv::Mat mod_image = img_core.multiply_saturation_color(image, 0.5, 0);
    img_core.display_image(1, mod_image);

    //img_core.get_DFT(image);

    waitKey(0); // Wait for a keystroke in the window
    return 0;
}