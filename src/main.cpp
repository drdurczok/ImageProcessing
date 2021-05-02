#include <opencv2/highgui.hpp>
#include <iostream>

#include "image_core.cpp"
#include "image_calibration.cpp"
#include "image_processing.cpp"

using namespace cv;
using namespace std;

int main(){
	//string path_to_image = "../samples/images/picture250.jpg";
	string path_to_image = "../calibration/homography.jpg";
	image_core img_core(2,2);
  	image_calibration img_calib;
	image_processing img_proc;
	
	Mat image = img_core.load_image(path_to_image);
	//img_core.display_image(0, image);
	//img_core.save_image("temp.jpg",image);

    img_calib.get_settings();
    img_core.display_image(0, img_proc.get_homography_origin_frame(image.clone()));

 	//Mat img_filtered = img_proc.filter(image);
	//Mat img_circles = img_proc.position_detection(img_filtered);

	/*Temporary code for testing */
	//Mat img_undistort = img_proc.undistort(image);
	//img_core.display_image(1, img_undistort);

 	img_core.display_image(1, img_proc.get_homography_frame(image));

    double p[] = {104,165};
 	cout << "\nPoint in camera coordinates: (" << p[0] << ", " << p[1] << ") \ngives the following point in world coordinates:\n" 
 		 << img_proc.homography_calc((Mat_<double>(3,1) << p[0], p[1], 1)) << endl;;

 	waitKey(0);
	/*End testing*/

  	//img_core.display_image(2, img_filtered);
 	//img_core.display_image(3, img_circles);




/*
 	Point2f center[2];
 	float radius[2];
 	center[0] = img_proc.getCircleCenter(0);
 	center[1] = img_proc.getCircleCenter(1);
 	radius[0] = img_proc.getCircleRadius(0);
 	radius[1] = img_proc.getCircleRadius(1);

  	for (uint i = 0; i < sizeof(radius)/sizeof(*radius); i++){
  		//cout << "circle " << i << " with center: " << center[i] << " radius: " << radius[i] << endl;
	}
	
    waitKey(0); // Wait for a keystroke in the window

    for (int i = 0; i < 300; i++){
    	//cout << "Distance to camera center [" << i << "]: " << img_proc.pixel_to_distance(i) << endl;
	}
*/
    return 0;
}

