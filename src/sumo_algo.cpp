#include "../inc/sumo_algo.hpp"

sumo_algo::sumo_algo(){

}

Mat sumo_algo::distance_to_edge(Mat image){
	Mat HFrame = img_proc.get_homography_frame(image);

	Mat img_filtered = img_proc.filter(HFrame);
	vector<Point2f> img_floor = img_proc.getFloorPixels_Points(img_filtered);

	//Find closest distance, increment by 10 pixels
	for (uint i = 0; i < img_floor.size(); i = i + 10){
		cout << "camera dist: " << img_proc.distance_camera_to_pixel(img_floor[i]) << endl;
	}

	img_proc.calculate_circle_dimensions(img_filtered);
  	cout << "Sumo ring with center: " << img_proc.getCircleCenter() << " radius: " << img_proc.getCircleRadius() << endl;
	
	return img_filtered;
}