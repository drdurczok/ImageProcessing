#include "../inc/sumo_edge.hpp"

sumo_edge::sumo_edge(){
	this->outer_ring_diameter_pixels = 1540 / this->pix_to_mm;
    this->outer_ring_radius_pixels = outer_ring_diameter_pixels / 2;

    this->success = false;
}

bool sumo_edge::find_robot_position(Mat image){
	this-> success = false;

	bool found_center = this->calculate_ring_center(image);
	
	if (found_center){
		this->robot.distance_to_center = this->distance_camera_to_pixel(this->circle_center);

		this->calculate_robot_position();

		this->success = true;
	}

	return this->success;
}

Point2f sumo_edge::get_robot_position(){
	return robot.coordinates;
}


/*_________________________ROBOT______________________________*/

void sumo_edge::calculate_robot_position(){
    //Find robot as a point
    Debug("Calculating robot position.");
    Point2f camera_point_homography = this->get_camera_coordinates();

    // Translate to Dohyo by computing the difference between real Dohyo center and acquired
    int cent_x = this->outer_ring_radius_pixels - this->circle_center.x;
    int cent_y = this->outer_ring_radius_pixels - this->circle_center.y;

    Point2f robot_point = Point2f(
    	cent_x + camera_point_homography.x ,
    	cent_y + camera_point_homography.y);

    robot.coordinates = robot_point;
}


/*______________________DOHYO CENTER___________________________*/

/*
 *	STEP 1	-	Calculate the slope of the tangent
 *	STEP 2	-	Calculate the offset of the tangent
 *	STEP 3	-	Calculate the normal through the point of the tangent
 *	STEP 4	-	Find center of circle
 */
bool sumo_edge::calculate_ring_center(Mat image){
	bool success = this->calculate_threshold_image(image, image);

	if (success){
		this->find_tangent(image);

		this->find_normal();

		this->find_circle_center();
	}
	else{
		CWARN("Threshold not found for image.");
		return false;
	}

	return true;
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

	Debug("Calculating tangent point.");
	//Redefine line as: ax + by + c = 0
	double m_a, m_b, m_c;
	m_a = this->line_tangent.at<double>(1,0);
	m_b = -1;
	m_c = this->line_tangent.at<double>(0,0);

	//Find point on top edge furthest from tangent
	double dist, dist_largest = 0;

	for (auto point : img_ceil){
		dist = abs(m_a*point.x + m_b*point.y + m_c) / sqrt(pow(m_a,2) + pow(m_b,2));

		if (dist > dist_largest){
			dist_largest = dist;
			this->tangent_point = point;
		}
	}
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
		CERR("Dividing by zero (line_tangent A).");
	}

	//Calculate offset of normal through tangent point
	this->line_normal.at<double>(0,0) = this->tangent_point.y - this->line_normal.at<double>(1,0) * this->tangent_point.x;
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

	int starting_thresh = 220;

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


		if (this->is_arc(contours, output)){
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

bool sumo_edge::is_arc(vector<vector<Point>> contours, Mat image ){
	// Find the rotated rectangles for each contour
    vector<RotatedRect> minRect(contours.size());
    double length_1, length_2, sagitta, width;
    double sagitta_1, sagitta_2;
    double sagitta_delta_1, sagitta_delta_2;
    double tolerance = 0.22 ;

    uint found = 0;

    for( int i = 0; i < contours.size(); i++ ){
        minRect[i] = minAreaRect( Mat(contours[i]) );

        // detect rectangle for each contour
        Point2f rect_points[4]; minRect[i].points( rect_points );

        length_1 = norm(Mat(rect_points[0]), Mat(rect_points[1]));
        length_2 = norm(Mat(rect_points[1]), Mat(rect_points[2]));

        if (length_1 > length_2){
        	sagitta = length_2;
        	width  = length_1;
        }
        else{
        	sagitta = length_1;
        	width  = length_2;
        }

        // sagitta - height of arc s = r +- sqrt(r^2 - l^2), where l is 1/2 the segment length
        sagitta_1 = this->outer_ring_radius_pixels + sqrt(pow(this->outer_ring_radius_pixels,2) - pow(width/2,2));
        sagitta_2 = this->outer_ring_radius_pixels - sqrt(pow(this->outer_ring_radius_pixels,2) - pow(width/2,2));

        sagitta_delta_1 = abs(sagitta - sagitta_1)/width;
        sagitta_delta_2 = abs(sagitta - sagitta_2)/width;


    	/* Draw fitted rectangle
    	Debug("Sagitta tolerance [% of width]: " + to_string(sagitta_delta_1) + " | " + to_string(sagitta_delta_2));
    	Mat temp = image.clone();
        minRect[i].points(rect_points);
        for (int j = 0; j < 4; j++)
        	line(temp, rect_points[j], rect_points[(j + 1) % 4], Scalar(255,0,0), 1, 8);
        imshow("DRAW", temp);
        waitKey(0);
        */

        if (sagitta_delta_1 < tolerance || sagitta_delta_2 < tolerance){
        	found++;
        }
        else{
        	return false;
        }
    }

    if (found <= 2){
    	return true;
    }

	//if (contours.size() == 1 || contours.size() == 2){return true;}

	return false;
}

/*______________________PREDICTIVE______________________*/
/*
 *	Creates new image that only includes the Dohyo surface and thresholds.
 */
Mat sumo_edge::isolate_dohyo(Mat image){
	Mat isolated_frame = Mat::zeros(image.size().height, image.size().width, CV_8U);

	uint avg_px_val = 140;

    if (this->success){
    	vector<Point> img_floor = assume_dohyo_inner_ring (image);

		for (auto point : img_floor){
			for (uint y = image.size().height-1; y > point.y; y--){
				isolated_frame.at<uint8_t>(y, point.x) = image.at<uint8_t>(y, point.x);
			}
		}

		threshold(isolated_frame, isolated_frame, avg_px_val, 255, THRESH_BINARY);

	}

	return isolated_frame;
}

/*
 *	(x - xc)^2 + (y - yc)^2 = r^2
 */
vector<Point> sumo_edge::assume_dohyo_inner_ring(Mat image){
	vector<Point> ring_floor;
	for (int x = 0; x < image.size().width; x++){	// y = yc - sqrt(r^2 - (x - xc)^2)
		double y = this->circle_center.y - sqrt(pow(outer_ring_radius_pixels-20/this->pix_to_mm,2) - pow((x - this->circle_center.x),2));
		ring_floor.push_back(Point(x, y));
	}
	return ring_floor;
}

vector<Point> sumo_edge::assume_dohyo_outer_ring(Mat image){
	vector<Point> ring_ceil;
	for (int x = 0; x < image.size().width; x++){	// y = yc - sqrt(r^2 - (x - xc)^2)
		double y = this->circle_center.y - sqrt(pow(outer_ring_radius_pixels,2) - pow((x - this->circle_center.x),2));
		ring_ceil.push_back(Point(x, y));
	}
	return ring_ceil;
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

    if (this->success){
	    this->calculate_robot_position();

	    circle(dohyo, robot.coordinates, 20, (0, 0, 255), -1);
		//namedWindow("dohyo", WINDOW_FREERATIO);	
	    //imshow("dohyo", dohyo);
	    //waitKey(0);

	    //cout << "Circle center:            " << this->circle_center << endl;
	    //cout << "Robot Point:              " << robot.coordinates   << endl;	

		//Draw viewing angle lines
		double viewing_angle = 72 * PI/180; //deg to rad
		double view_ang_1 = PI/2 + viewing_angle/2;
		double view_ang_2 = PI/2 - viewing_angle/2;

		vector<Point> line_view_1_points = this->get_fov_line_points(dohyo, robot.coordinates, view_ang_1);
		polylines(dohyo, line_view_1_points, false, Scalar(255, 0, 0), 2, 8);

		vector<Point> line_view_2_points = this->get_fov_line_points(dohyo, robot.coordinates, view_ang_2);
		polylines(dohyo, line_view_2_points, false, Scalar(255, 0, 0), 2, 8);

		ostringstream debug_msg;
		debug_msg << "Tangent: y = " << line_tangent.at<double>(1,0) <<  "x + " << line_tangent.at<double>(0,0);
		Debug(debug_msg.str());
		debug_msg.clear(); debug_msg.str("");

		debug_msg << "Normal: y =  " << this->line_normal.at<double>(1,0) << "x + " << this->line_normal.at<double>(0,0);
		Debug(debug_msg.str());
		debug_msg.clear(); debug_msg.str("");

		debug_msg << "Tangent point: " << this->tangent_point << " [x, y]";
		Debug(debug_msg.str());
		debug_msg.clear(); debug_msg.str("");

		debug_msg << "Robot point:   " << this->robot.coordinates << " [x, y]";
		Debug(debug_msg.str());

		Debug("Robot distance to center: " + to_string(this->robot.distance_to_center/10) + " cm");
	}

    return dohyo;
}

Mat sumo_edge::draw_lines(Mat frame){
	if (this->success){
		vector<Point> line_tangent_points = this->get_line_points(frame, this->line_tangent);
		polylines(frame, line_tangent_points, false, Scalar(255, 0, 0), 2, 8);

		vector<Point> line_normal_points = this->get_line_points(frame, this->line_normal);
		polylines(frame, line_normal_points, false, Scalar(255, 0, 0), 2, 8);

	    circle(frame, this->tangent_point, 5, (0, 0, 255), -1);
	}

	return frame;
}