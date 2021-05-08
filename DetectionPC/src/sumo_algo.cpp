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

Mat sumo_algo::distance_to_center(Mat image){
	Mat HFrame = img_proc.get_homography_frame(image);

	Mat img_filtered = img_proc.filter(HFrame);

	Mat img_circles = img_proc.calculate_circle_dimensions(img_filtered);

	double dist = img_proc.distance_camera_to_pixel(img_proc.getCircleCenter());

  	cout << "Sumo ring with center:  " << img_proc.getCircleCenter() << " radius: " << img_proc.getCircleRadius() << endl;
    cout << "Camera coordinates are: " << img_proc.get_camera_coordinates() << endl;
  	cout << "Camera is " << dist << "mm away from ring center" << endl;

	return img_circles;
}

Mat sumo_algo::calculate_ring_center(Mat image){
	Mat HFrame = img_proc.get_homography_frame(image);
	Mat img_filtered = img_proc.filter(HFrame);
	vector<Point2f> img_floor = img_proc.getFloorPixels_Points(img_filtered);
	vector<Point2f> img_ceil = img_proc.getCeilingPixels_Points(img_filtered);


	//START FIND TANGENT

	//Definition of both lines, y = ax + b
 	Mat line_floor = this->TotalLeastSquares(img_floor);
	Mat line_ceil = this->TotalLeastSquares(img_ceil);

	Mat line_tangent = Mat::zeros(2, 2, CV_64F);
	double tolerance = 0.1;

	if (abs(line_floor.at<double>(1, 0) - line_ceil.at<double>(1, 0)) < tolerance ){
		line_tangent.at<double>(1,0) = (line_floor.at<double>(1, 0) + line_ceil.at<double>(1, 0)) / 2;
		line_tangent.at<double>(0,0) = (line_floor.at<double>(0, 0) + line_ceil.at<double>(0, 0)) / 2;
	}

	// Find b element by raising the average of the two calculated lines
	vector<Point> line_tangent_points = this->get_line_points(HFrame, line_tangent);

	// Find where the highest point of the line in the image to avoid overflow
	uint end_of_line = 0;
	if (line_tangent_points[0].y < line_tangent_points[line_tangent_points.size()-1].y){
		end_of_line = line_tangent_points[0].y;
	}
	else{
		end_of_line = line_tangent_points[line_tangent_points.size()-1].y;
	}

	uint pixels_found_count;
	uint tangent_shift;
	Point2f 		tangent_point;
	vector<Point2f> tangent_points;
	vector<Point2f> tangent_points_buffer;

	for (uint i = 0; i < end_of_line; i++){
		pixels_found_count = 0;

		for (uint x = 0; x < line_tangent_points.size()-1; x++){
			if (img_filtered.at<uint>(x, (line_tangent_points[x].y - i)) != 0){
				tangent_points_buffer.push_back(Point2f(x, (line_tangent_points[x].y - i)));
				pixels_found_count++;
			}
		}

		if (pixels_found_count == 0){
			tangent_shift = i;
			break;
		}
		else if (pixels_found_count < 5){
			tangent_points = tangent_points_buffer;
			tangent_shift = i;
			break;
		}
		else{
			tangent_points = tangent_points_buffer;
			tangent_points_buffer.clear();
		}
	}
	tangent_point = tangent_points[ tangent_points.size()/2 ];

	//cout << "tangent point\n" << tangent_point << endl;

	line_tangent.at<double>(0,0) = line_tangent.at<double>(0,0) - tangent_shift;

	//END FIND TANGENT


	//START FIND NORMAL

	Mat line_normal = Mat::zeros(2, 2, CV_64F);
	if (line_tangent.at<double>(1,0) != 0){
		line_normal.at<double>(1,0) = -1 / line_tangent.at<double>(1,0);
	}
	else {
		cout << "ERROR: Dividing by zero (line_tangent A)." << endl;
	}

	line_normal.at<double>(0,0) = tangent_point.y - line_normal.at<double>(1,0) * tangent_point.x;

	//END FIND NORMAL


	//START FIND CIRCLE CENTER

	double outer_ring_diameter_pixels = 1560;
    double outer_ring_radius_pixels = outer_ring_diameter_pixels / 2;

    Point2f circle_center;

    circle_center.x = tangent_point.x + sqrt(pow(outer_ring_radius_pixels,2) / (1 + pow(line_normal.at<double>(1,0),2)));
    circle_center.y = circle_center.x * line_normal.at<double>(1,0) + line_normal.at<double>(0,0);

    cout << "Circle center: " << circle_center << endl;

	//END FIND CIRCLE CENTER


	//START DISTANCE TO ROBOT

	double distance_to_center = img_proc.distance_camera_to_pixel(circle_center);

	cout << "Robot distance to center: " << distance_to_center/10 << " cm" << endl;

	//END DISTANCE TO ROBOT



	//START DRAW ROBOT
	string path_to_image = "../calibration/Dohyo.jpg";
    Mat dohyo = imread(path_to_image, IMREAD_COLOR);

    Point2f camera_point_homography = img_proc.get_camera_coordinates();

    int cent_x = outer_ring_diameter_pixels/2 - circle_center.x;
    int cent_y = outer_ring_diameter_pixels/2 - circle_center.y;

    Point2f robot_point = Point2f(
    	cent_x + camera_point_homography.x ,
    	cent_y + camera_point_homography.y);

    cout << "Robot Point: " << robot_point << endl;	

    circle(dohyo, robot_point, 20, (0, 0, 255), -1);


	namedWindow("dohyo", WINDOW_FREERATIO);	
    imshow("dohyo", dohyo);
    waitKey(0);


	//END DRAW ROBOT


	//Draw tangent
	line_tangent_points = this->get_line_points(HFrame, line_tangent);
	polylines(HFrame, line_tangent_points, false, Scalar(255, 0, 0), 1, 8);
	//Draw normal
	vector<Point> line_normal_points = this->get_line_points(HFrame, line_normal);
	polylines(HFrame, line_normal_points, false, Scalar(255, 0, 0), 1, 8);

	return HFrame;

}

Mat sumo_algo::TotalLeastSquares(vector<Point2f> points) {
	//Build A matrix 
	int N = 2;
	Mat A = Mat::zeros(N, N, CV_64FC1);
 
	for (int row = 0; row < A.rows; row++){
		for (int col = 0; col < A.cols; col++){
			for (int k = 0; k < points.size(); k++){
				A.at<double>(row, col) = A.at<double>(row, col) + pow(points[k].x, row + col);
			}
		}
	}
	 //Build B matrix
	Mat B = Mat::zeros(N, 1, CV_64FC1);
	for (int row = 0; row < B.rows; row++){
		for (int k = 0; k < points.size(); k++){
			B.at<double>(row, 0) = B.at<double>(row, 0) + pow(points[k].x, row)*points[k].y;
		}
	}

	//A*X=B
	Mat X;
	solve(A, B, X, DECOMP_LU);

	//cout << "y = b + ax" << endl;
	//cout << "y = " << X.at<double>(0, 0) << " + " << X.at<double>(1, 0) << " x" << endl;

	return X;
}

vector<Point> sumo_algo::get_line_points(Mat image, Mat X){
	vector<Point>lines;
	for (int x = 0; x < image.size().width; x++){				// y = b + ax;
		double y = X.at<double>(0, 0) + X.at<double>(1, 0)*x;
		lines.push_back(Point(x, y));
	}

    return lines;
}