#include <opencv2/highgui.hpp>
#include <iostream>

#include "image_core.cpp"

using namespace cv;
using namespace std;

int main(){
	image_core img_core;
	Mat image = img_core.get_image();

    namedWindow( "Display window", WINDOW_AUTOSIZE ); // Create a window for display.
    imshow( "Display window", image ); // Show our image inside it.
    waitKey(0); // Wait for a keystroke in the window
    return 0;
}