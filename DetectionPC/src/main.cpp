#include <opencv2/highgui.hpp>
#include <iostream>

#include "image_core.cpp"
#include "image_processing.cpp"
#include "sumo_algo.cpp"

//Measure system time
#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration;

using namespace cv;
using namespace std;

image_core img_core(2,2);
sumo_algo sumo;

void example_03();
void example_04();
void example_05();

uint num_of_images = 35;

int main(){
	auto t_start = high_resolution_clock::now();

	//example_03();
	example_04();

  	auto t_end = high_resolution_clock::now();

    /* Getting number of milliseconds as a double. */
    duration<double, std::milli> ms_double = t_end - t_start;
  	cout << "Execution time total:         " << ms_double.count() << "ms" << endl;
  	cout << "Execution time avg per image: " << ms_double.count()/num_of_images << "ms" << endl;

 	waitKey(0);

    return 0;
}


void esp_main(){
	//Take picture
	Mat image;

	Point2f robot_position = sumo.find_robot_position(image);
  	
	//Send to STM32 through uart
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

/* example_03
 *	Process single image and display
 */
void example_03(){
	uint image_i = 6;

	String path_to_image = "../samples/" + to_string(image_i) + ".jpg";
	//string path_to_image = "../calibration/homography.jpg";

	Mat image = img_core.load_image(path_to_image);
	sumo.calculate_ring_center(image);
	sumo.calculate_robot_position();
	
	Mat HFrame_processed = sumo.draw_homography_frame(image);
	Mat Dohyo = sumo.draw_dohyo();
  	
  	img_core.display_image(0, image);
  	img_core.display_image(1, HFrame_processed);
  	img_core.display_image(3, Dohyo);

 	waitKey(0);
}


/* example_04
 *	Process multiple images and display
 */
void example_04(){
	string path_to_image;
	string path_to_results;

	Mat image;
	Mat HFrame_processed;
	Mat Dohyo;

	for (uint i = 1; i < num_of_images; i++){
		cout << "Image: " << i << endl;
		path_to_image = "../samples/" + to_string(i) + ".jpg";

		image = img_core.load_image(path_to_image);
		sumo.calculate_ring_center(image);
		sumo.calculate_robot_position();
		
		HFrame_processed = sumo.draw_homography_frame(image);
		Dohyo = sumo.draw_dohyo();
	  	
	  	img_core.display_image(0, image);
	  	img_core.display_image(1, HFrame_processed);
	  	img_core.display_image(3, Dohyo);

	 	waitKey(0);
  	}
}

/* example_05
 *	Process multiple images and save
 */
void example_05(){
	string path_to_image;
	string path_to_results;

	Mat image;
	Mat HFrame_processed;
	Mat Dohyo;

	for (uint i = 1; i < num_of_images; i++){
		cout << "Image: " << i << endl;
		path_to_image = "../samples/" + to_string(i) + ".jpg";

		image = img_core.load_image(path_to_image);
		sumo.calculate_ring_center(image);
		sumo.calculate_robot_position();

		HFrame_processed = sumo.draw_homography_frame(image);
		Dohyo = sumo.draw_dohyo();

		path_to_results = "../results/" + to_string(i) + ".jpg";
	  	img_core.save_image(path_to_results, Dohyo);
  	}
}