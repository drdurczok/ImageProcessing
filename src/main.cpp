#include <opencv2/highgui.hpp>
#include <iostream>

#include "image_core.cpp"
#include "image_calibration.cpp"
#include "image_processing.cpp"
#include "sumo_algo.cpp"

using namespace cv;
using namespace std;

image_core img_core(2,2);
image_calibration img_calib;
sumo_algo sumo;

int main(){
	string path_to_image = "../samples/images/101.jpg";
	//string path_to_image = "../calibration/homography.jpg";

	Mat image = img_core.load_image(path_to_image);

	Mat HFrame_processed = sumo.distance_to_edge(image);
  	
  	img_core.display_image(0, image);
  	img_core.display_image(1, HFrame_processed);


 	waitKey(0);

    return 0;
}




//EXAMPLES

/* example_01
 *	This example takes any point on the original image and calculates the point on the homography.
 *	Then it calculates all points of the image into homography coordinates
 *	Lastly it outputs the real world distance in mm of the point to the camera normal to the homography plane.
 */
void example_01(Mat image){
	image_processing img_proc;	

	// Choose any point on the image (x,y)
	Point2f CPoint = Point2f(133, 50);
	// Calculate the point on the homography
    Point2f HPoint = img_proc.homography_calc(CPoint);

    // Add and display the point and coordinate system on original image.
    Mat frame_with_origin = img_proc.get_homography_origin_frame(image.clone());
    img_core.display_image(0, img_proc.draw_point_on_frame(frame_with_origin, CPoint));

    // Get the homography transform off original image (calculate ALL points for image)
    Mat HFrame = img_proc.get_homography_frame(image);

    // Display homography with calculated point
 	img_core.display_image(1, img_proc.draw_point_on_frame(HFrame, HPoint));


 	cout << "\nPoint in camera coordinates: " << CPoint << "\ngives the following point in world coordinates:\n" 
 		 << HPoint << endl;

 	cout << "\nDistance to pixel from camera: " << img_proc.distance_camera_to_pixel(CPoint) << endl;
}

/* example_02
 *	Load, display and save image.
 */
void example_02(){
	string path_to_image = "../samples/images/picture250.jpg";
	Mat image = img_core.load_image(path_to_image);
	
	img_core.display_image(1, image);

	img_core.save_image("temp.jpg", image);
}