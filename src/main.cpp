#include <opencv2/highgui.hpp>
#include <iostream>

#include "image_core.cpp"
#include "image_processing.cpp"

using namespace cv;
using namespace std;


int main(){
	string path_to_image = "../samples/images/IMG_003.jpg";
	image_core img_core(2,2);
	image_processing img_proc;
	
	Mat image = img_core.load_image(path_to_image);
	img_core.display_image(0, image);
	//img_core.save_image("temp.jpg",image);

	//Mat mod_image = img_core.multiply_saturation_color(image, 0.5, 0);
    //img_core.display_image(1, mod_image);

    //img_core.get_DFT(image);

    Mat img_filtered = img_proc.filter(image);
    Mat img_detected = img_proc.find_edge(img_filtered, img_proc.SOBEL);
	Mat img_contours = img_proc.ellipse_detection(img_detected);

    img_core.display_image(1, img_filtered);
   	img_core.display_image(2, img_detected);
   	img_core.display_image(3, img_contours);


    waitKey(0); // Wait for a keystroke in the window
    return 0;
}