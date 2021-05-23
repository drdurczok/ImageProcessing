#include "../inc/sumo_edge.hpp"

sumo_edge::sumo_edge(){
	outer_ring_diameter_pixels = 1550;
    outer_ring_radius_pixels = outer_ring_diameter_pixels / 2;
}

Point2f sumo_edge::find_robot_position(Mat image){
	this->calculate_ring_center(image);

	this->robot.distance_to_center = this->distance_camera_to_pixel(this->circle_center);

	Point2f robot_point = this->calculate_robot_position();

	return robot_point;
}

/*_________________________ROBOT______________________________*/

Point2f sumo_edge::calculate_robot_position(){
    //Find robot as a point
    Debug("Calculating robot position.");
    Point2f camera_point_homography = this->get_camera_coordinates();

    int cent_x = this->outer_ring_radius_pixels - this->circle_center.x;
    int cent_y = this->outer_ring_radius_pixels - this->circle_center.y;

    Point2f robot_point = Point2f(
    	cent_x + camera_point_homography.x ,
    	cent_y + camera_point_homography.y);

    return robot_point;
}


/*______________________DOHYO CENTER___________________________*/

/*
 *	STEP 1	-	Calculate the slope of the tangent
 *	STEP 2	-	Calculate the offset of the tangent
 *	STEP 3	-	Calculate the normal through the point of the tangent
 *	STEP 4	-	Find center of circle
 */
void sumo_edge::calculate_ring_center(Mat image){
	bool success = this->calculate_threshold_image(image, image);

	if (success){
		this->find_tangent(image);

		this->find_normal();

		this->find_circle_center();
	}
	else{
		CWARN("Threshold not found for image.");
	}
}

/*
 *	Calculate the slope of the tangent
 *		The tangent slope is calculated by using the least squares method.
 *		The average of the filtered floor and ceiling points provide us with a fairly accurate value.
 *	Calculate the offset of the tangent
 *		The offset is calculated by raising the tangent untill no more pixels are found above it.
 */
void sumo_edge::find_tangent(Mat img){
	vector<Point2f> img_floor = this->getFloorPixels_Points(img);
	vector<Point2f> img_ceil = this->getCeilingPixels_Points(img);

	Debug("Calculating tangent slope.");
	//Definition of both lines, y = ax + b
 	Mat line_floor = this->TotalLeastSquares(img_floor);
	Mat line_ceil = this->TotalLeastSquares(img_ceil);

	this->line_tangent = Mat::zeros(2, 2, CV_64F);
	double tolerance = 0.1;

	if (abs(line_floor.at<double>(1, 0) - line_ceil.at<double>(1, 0)) < tolerance ){
		this->line_tangent.at<double>(1,0) = (line_floor.at<double>(1, 0) + line_ceil.at<double>(1, 0)) / 2;
		this->line_tangent.at<double>(0,0) = (line_floor.at<double>(0, 0) + line_ceil.at<double>(0, 0)) / 2;
	}
	else{
		CWARN("The calculated tangents have a large disparity. Defaulting to floor values.");
		this->line_tangent.at<double>(1,0) = line_floor.at<double>(1, 0);
		this->line_tangent.at<double>(0,0) = line_floor.at<double>(0, 0);
	}

	vector<Point> line_tangent_points = this->get_line_points(img, this->line_tangent);

	//Remove points that are out of image frame from top
	for (uint i = 0; i < line_tangent_points.size(); i++){
		if(line_tangent_points[i].y < 0){
			line_tangent_points.erase(line_tangent_points.begin());
			i--;
		}
		else{
			break;
		}
	}

	//Remove points that are out of image frame from bottom
	for (uint i = line_tangent_points.size()-1; i > 0; i--){
		if(line_tangent_points[i].y > img.size().height){
			line_tangent_points.erase(line_tangent_points.end());
		}
		else{
			break;
		}
	}

	/*Debug Draw Tangent Slope
	cout << "Tangent: y = " << line_tangent.at<double>(1,0) << "x + " << line_tangent.at<double>(0,0) << endl;
	polylines(img, line_tangent_points, false, Scalar(255, 0, 0), 2, 8);
	imshow("Tangent", img);
	waitKey(0);
	*/
	
	Debug("Calculating tangent offset.");
	uint pixels_found_count = 0;
	uint tangent_shift = 0;
	uint x, y;
	vector<Point> tangent_points;
	vector<Point> tangent_points_buffer;

	while(true){
		pixels_found_count = 0;

		//Iterate through all points on line in image
		for (uint i = 0; i < line_tangent_points.size(); i++){
			//Condition to avoid image overflow
			if (line_tangent_points[i].y > tangent_shift){
				//Calculate image coordinates
				x = line_tangent_points[i].x;
				y = (line_tangent_points[i].y - tangent_shift);
				//cout << "tan: " << line_tangent_points[i].y << " shift: " << tangent_shift << endl;
				//cout << "x: " << x << ", y: " << y << endl;
				//Check value of point
				if (img.at<char>(y, x) != 0){
					tangent_points_buffer.push_back(Point2f(x, y));
					pixels_found_count++;
				}
			}
		}

		if (pixels_found_count == 0){
			break;
		}
		else if (pixels_found_count < 5){
			tangent_points = tangent_points_buffer;
			break;
		}
		else{
			tangent_points = tangent_points_buffer;
			tangent_points_buffer.clear();
		}

		tangent_shift++;
	}

	this->tangent_point = tangent_points[ tangent_points.size()/2 ];

	this->line_tangent.at<double>(0,0) = this->line_tangent.at<double>(0,0) - tangent_shift;

	/*Debug Draw Tangent
	cout << "Tangent point: " << this->tangent_point << " [x, y]" << endl;
	cout << "Tangent: y = " << line_tangent.at<double>(1,0) << "x + " << line_tangent.at<double>(0,0) << endl;
	line_tangent_points = this->get_line_points(img, line_tangent);
	polylines(img, line_tangent_points, false, Scalar(255, 0, 0), 2, 8);
	imshow("Tangent", img);
	waitKey(0);
	*/
	
}

/*
 *	Calculate the normal through the point of the tangent
 *		Slope  = - 1 / Tangent_slope
 *		Offset = y - Slope * x
 */
void sumo_edge::find_normal(){
	Debug("Calculating normal.");
	this->line_normal = Mat::zeros(2, 2, CV_64F);

	//Condition to avoid dividing by zero
	if (this->line_tangent.at<double>(1,0) != 0){
		//Calculate slope of normal
		this->line_normal.at<double>(1,0) = -1 / this->line_tangent.at<double>(1,0);
	}
	else {
		cout << "ERROR: Dividing by zero (line_tangent A)." << endl;
	}

	//Calculate offset of normal through tangent point
	this->line_normal.at<double>(0,0) = this->tangent_point.y - this->line_normal.at<double>(1,0) * this->tangent_point.x;

	/*Debug Draw Normal
	cout << "Normal: y = " << this->line_normal.at<double>(1,0) << "x + " << this->line_normal.at<double>(0,0) << endl;
	vector<Point> line_normal_points = this->get_line_points(HFrame, this->line_normal);
	polylines(HFrame, line_normal_points, false, Scalar(255, 0, 0), 2, 8);
	imshow("Tangent", HFrame);
	waitKey(0);
	*/
}

/*
 *	Find center of circle
 *		The center of the circle is calculated by finding the point from the tangent point that is
 *		a distance of the radius away along the normal line.
 *		x1 = x0 +- sqrt( distance^2 / (1 + slope^2) ) 
 *		y1 = slope * x1 + offset
 */
void sumo_edge::find_circle_center(){
    double sqr = sqrt(pow(this->outer_ring_radius_pixels,2) / (1 + pow(this->line_normal.at<double>(1,0),2)));

	this->circle_center.x = this->tangent_point.x - sqr;
    this->circle_center.y = this->circle_center.x * this->line_normal.at<double>(1,0) + this->line_normal.at<double>(0,0);
    if (this->circle_center.y < 0){
    	this->circle_center.x = this->tangent_point.x + sqr;
        this->circle_center.y = this->circle_center.x * this->line_normal.at<double>(1,0) + this->line_normal.at<double>(0,0);
    }	
}


/*______________________ALGORITHMS_____________________________*/

Mat sumo_edge::TotalLeastSquares(vector<Point2f> points) {
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

vector<Point> sumo_edge::get_line_points(Mat image, Mat X){
	vector<Point>lines;
	for (int x = 0; x < image.size().width; x++){				// y = b + ax;
		double y = X.at<double>(0, 0) + X.at<double>(1, 0)*x;
		lines.push_back(Point(x, y));
	}

    return lines;
}

vector<Point> sumo_edge::get_fov_line_points(Mat image, Point2f point, double angle){
	Mat line = Mat::zeros(2, 2, CV_64F);
	line.at<double>(1,0) = tan(angle);
	line.at<double>(0,0) = point.y - line.at<double>(1,0) * point.x;

	vector<Point> line_points;
	if (line.at<double>(1,0) < 0){
		for (int x = point.x; x < image.size().width; x++){	// y = b + ax;
			double y = line.at<double>(0, 0) + line.at<double>(1, 0)*x;
			line_points.push_back(Point(x, y));
		}
	}
	else{
		for (int x = point.x; x > 0; x--){					// y = b + ax;
			double y = line.at<double>(0, 0) + line.at<double>(1, 0)*x;
			line_points.push_back(Point(x, y));
		}
	}

	return line_points;
}

/*______________________THRESHOLD___________________________*/

bool sumo_edge::calculate_threshold_image(Mat input, Mat & result){
	uint min_threshold;
	uint max_threshold = 255;

	Mat kernel_close = getStructuringElement(MORPH_RECT, Size(3,3));
	Mat kernel_open  = getStructuringElement(MORPH_RECT, Size(5,5));

	vector<vector<Point>> contours;
	double area;
	double area_prev = 999999;
	double min_area = 4000;
	double max_area = 20000;
	double max_step_area = 10000;
	double delta_step_area = 0;
	double delta_step_area_prev = 0;

	int starting_thresh = 190;

	Mat output;

	int step = 15;
	for (int min_threshold = starting_thresh; min_threshold > 0 ; min_threshold -= step){
		threshold(input, output, min_threshold, max_threshold, THRESH_BINARY);

		/* Closing morphology, removes small holes */
		morphologyEx(output, output, MORPH_CLOSE, kernel_close);

		/* Opening morphology, removes small objects */
		morphologyEx(output, output, MORPH_OPEN, kernel_open);

		GaussianBlur(output, output, Size(3,3), 1);

		findContours(output, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
		
		/*	Display Contours 
		Mat temp = input.clone();
		drawContours(temp, contours, -1, Scalar(0,255,75), 2);
		imshow("a", temp);
		waitKey(0);
		*/

		if (contours.size() == 1){
			area = contourArea(contours[0]);

			if (area > max_area){break;}
			if (area > min_area){
				if (area - area_prev > max_step_area){break;}
			}

			if (delta_step_area > 0){
				delta_step_area = area - area_prev;
				if (delta_step_area > delta_step_area_prev){break;}
			}
			else{
				delta_step_area = area;
			}

			area_prev = area;
			delta_step_area_prev = delta_step_area;
		}
		else if (min_threshold == starting_thresh){
			return false;
		}
		else{
			break;
		}

		result = output.clone();
	}

	if(area_prev < 999999){
		Debug("Min threshold: " + to_string(min_threshold));
		Debug("Area:          " + to_string(area_prev));
		return true;
	}
	else{
		CWARN("Found no contours");
		return false;
	}
}

/*______________________VISUAL___________________________*/

Mat sumo_edge::draw_homography_frame(Mat image){
	Mat HFrame = this->get_homography_frame(image);

	//Draw tangent
	vector<Point> line_tangent_points = this->get_line_points(HFrame, this->line_tangent);
	polylines(HFrame, line_tangent_points, false, Scalar(255, 0, 0), 2, 8);
	//Draw normal
	vector<Point> line_normal_points = this->get_line_points(HFrame, this->line_normal);
	polylines(HFrame, line_normal_points, false, Scalar(255, 0, 0), 2, 8);

	return HFrame;
}

Mat sumo_edge::draw_dohyo(){
	string path_to_image = "../calibration/Dohyo.jpg";
    Mat dohyo = imread(path_to_image, IMREAD_COLOR);

    Point2f robot_point= this->calculate_robot_position();

    circle(dohyo, robot_point, 20, (0, 0, 255), -1);
	//namedWindow("dohyo", WINDOW_FREERATIO);	
    //imshow("dohyo", dohyo);
    //waitKey(0);

    //cout << "Circle center:            " << this->circle_center << endl;
    //cout << "Robot Point:              " << robot_point   << endl;	
    Debug("Robot distance to center: " + to_string(this->robot.distance_to_center/10) + " cm");

	//Draw viewing angle lines
	double viewing_angle = 40 * PI/180; //deg to rad
	double view_ang_1 = PI/2 + viewing_angle/2;
	double view_ang_2 = PI/2 - viewing_angle/2;

	vector<Point> line_view_1_points = this->get_fov_line_points(dohyo, robot_point, view_ang_1);
	polylines(dohyo, line_view_1_points, false, Scalar(255, 0, 0), 2, 8);

	vector<Point> line_view_2_points = this->get_fov_line_points(dohyo, robot_point, view_ang_2);
	polylines(dohyo, line_view_2_points, false, Scalar(255, 0, 0), 2, 8);

    return dohyo;
}