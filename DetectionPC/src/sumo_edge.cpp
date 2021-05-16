#include "../inc/sumo_edge.hpp"

sumo_edge::sumo_edge(){
}

Point2f sumo_edge::find_robot_position(Mat image){
	this->calculate_ring_center(image);
	Point2f robot_point = this->calculate_robot_position();

	return robot_point;
}

/*
 *	STEP 1	-	Calculate the slope of the tangent
 *		The tangent slope is calculated by using the least squares method.
 *		The average of the filtered floor and ceiling points provide us with a fairly accurate value.
 *	STEP 2	-	Calculate the offset of the tangent
 *		The offset is calculated by raising the tangent untill no more pixels are found above it.
 *	STEP 3	-	Calculate the normal through the point of the tangent
 *		Slope  = - 1 / Tangent_slope
 *		Offset = y - Slope * x
 *	STEP 4	-	Find center of circle
 *		The center of the circle is calculated by finding the point from the tangent point that is
 *		a distance of the radius away along the normal line.
 *		x1 = x0 +- sqrt( distance^2 / (1 + slope^2) ) 
 *		y1 = slope * x1 + offset
 */
void sumo_edge::calculate_ring_center(Mat image){
	Mat HFrame = img_proc.get_homography_frame(image);
	Mat img_filtered;
	bool success = img_proc.filter(HFrame, img_filtered, 0);

	if (success){
		vector<Point2f> img_floor = img_proc.getFloorPixels_Points(img_filtered);
		vector<Point2f> img_ceil = img_proc.getCeilingPixels_Points(img_filtered);

		//START FIND TANGENT
		#ifdef DEBUG
		cout << "INFO: Calculating tangent slope." << endl;
		#endif
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
			#ifdef DEBUG
			cout << "WARNING: The calculated tangents have a large disparity. Defaulting to floor values." << endl;
			#endif
			this->line_tangent.at<double>(1,0) = line_floor.at<double>(1, 0);
			this->line_tangent.at<double>(0,0) = line_floor.at<double>(0, 0);
		}

		vector<Point> line_tangent_points = this->get_line_points(HFrame, this->line_tangent);

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
			if(line_tangent_points[i].y > HFrame.size().height){
				line_tangent_points.erase(line_tangent_points.end());
			}
			else{
				break;
			}
		}

		/*Debug Draw Tangent Slope
		cout << "Tangent: y = " << line_tangent.at<double>(1,0) << "x + " << line_tangent.at<double>(0,0) << endl;
		polylines(HFrame, line_tangent_points, false, Scalar(255, 0, 0), 2, 8);
		imshow("Tangent", HFrame);
		waitKey(0);
		*/
		#ifdef DEBUG
		cout << "INFO: Calculating tangent offset." << endl;
		#endif
		uint pixels_found_count = 0;
		uint tangent_shift = 0;
		uint x, y;
		Point		  tangent_point;
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
					if (img_filtered.at<char>(y, x) != 0){
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

		tangent_point = tangent_points[ tangent_points.size()/2 ];

		this->line_tangent.at<double>(0,0) = this->line_tangent.at<double>(0,0) - tangent_shift;

		/*Debug Draw Tangent
		cout << "Tangent point: " << tangent_point << " [x, y]" << endl;
		cout << "Tangent: y = " << line_tangent.at<double>(1,0) << "x + " << line_tangent.at<double>(0,0) << endl;
		line_tangent_points = this->get_line_points(HFrame, line_tangent);
		polylines(HFrame, line_tangent_points, false, Scalar(255, 0, 0), 2, 8);
		imshow("Tangent", HFrame);
		waitKey(0);
		*/
		//END FIND TANGENT


		//START FIND NORMAL
		#ifdef DEBUG
		cout << "INFO: Calculating normal." << endl;
		#endif
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
		this->line_normal.at<double>(0,0) = tangent_point.y - this->line_normal.at<double>(1,0) * tangent_point.x;

		/*Debug Draw Normal
		cout << "Normal: y = " << this->line_normal.at<double>(1,0) << "x + " << this->line_normal.at<double>(0,0) << endl;
		vector<Point> line_normal_points = this->get_line_points(HFrame, this->line_normal);
		polylines(HFrame, line_normal_points, false, Scalar(255, 0, 0), 2, 8);
		imshow("Tangent", HFrame);
		waitKey(0);
		*/
		//END FIND NORMAL


		//START FIND CIRCLE CENTER 
	    double sqr = sqrt(pow(this->outer_ring_radius_pixels,2) / (1 + pow(this->line_normal.at<double>(1,0),2)));

		this->circle_center.x = tangent_point.x - sqr;
	    this->circle_center.y = this->circle_center.x * this->line_normal.at<double>(1,0) + this->line_normal.at<double>(0,0);
	    if (this->circle_center.y < 0){
	    	this->circle_center.x = tangent_point.x + sqr;
	        this->circle_center.y = this->circle_center.x * this->line_normal.at<double>(1,0) + this->line_normal.at<double>(0,0);
	    }
		//END FIND CIRCLE CENTER


		//START DISTANCE TO ROBOT
		this->robot.distance_to_center = img_proc.distance_camera_to_pixel(this->circle_center);
		//END DISTANCE TO ROBOT
	}
	else{
		cout << "WARNING: Threshold not found for image." << endl;
	}
}

Point2f sumo_edge::calculate_robot_position(){
    //Find robot as a point
    #ifdef DEBUG
    cout << "INFO: Calculating robot position." << endl;
    #endif
    Point2f camera_point_homography = img_proc.get_camera_coordinates();

    int cent_x = this->outer_ring_radius_pixels - this->circle_center.x;
    int cent_y = this->outer_ring_radius_pixels - this->circle_center.y;

    Point2f robot_point = Point2f(
    	cent_x + camera_point_homography.x ,
    	cent_y + camera_point_homography.y);

    return robot_point;
}


/*______________________ALGORITHMS___________________________*/

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

/*______________________VISUAL___________________________*/
Mat sumo_edge::draw_homography_frame(Mat image){
	Mat HFrame = img_proc.get_homography_frame(image);

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
    #ifdef DEBUG
    cout << "Robot distance to center: " << this->robot.distance_to_center/10 << " cm" << endl;
    #endif
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