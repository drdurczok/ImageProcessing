#include "../inc/main.hpp"

int main(){
	namedWindow("Dohyo", cv::WINDOW_FREERATIO);
	resizeWindow("Dohyo", 500, 500);

	run();
}

void run(){
	Point2f robot_position, opponent_position;
	double opponent_angle;

	string line;
	ifstream file ("../results/UART_FEED.txt");

	unsigned first, last;
	string RX,RY,OX,OY,OA;

	if (file.is_open()){
		while (getline(file,line)){
			//cout << line << endl;
			
			if (line.find("RX") != string::npos){
				first = line.find("RX");
				last = line.find("RY");
				RX = line.substr(first+2, last-first);

				first = line.find("RY");
				last = line.find("OX");
				RY = line.substr(first+2, last-first);

				robot_position = Point2f(atof(RX.c_str()), atof(RY.c_str()));
				//cout << "Robot Positino: " << robot_position << endl;

				draw_dohyo(robot_position);
			}

			if (line.find("OX") != string::npos){
				first = line.find("OX");
				last = line.find("OY");
				OX = line.substr(first+2, last-first);

				first = line.find("OY");
				last = line.find("OA");
				OY = line.substr(first+2, last-first);

				first = line.find("OA");
				last = line.size()-1;
				OA = line.substr(first+2, last-first);

				opponent_position = Point2f(atof(OX.c_str()), atof(OY.c_str()));
				opponent_angle    = atof(OA.c_str());
			}
		}

		file.close();
	}
	else cout << "Unable to open file"; 

}

void draw_dohyo(Point2f robot_position){
	string path_to_image = "../Dohyo.jpg";
    Mat dohyo = imread(path_to_image, IMREAD_COLOR);

    //Draw robot position
    circle(dohyo, robot_position, 20, (0, 0, 255), -1);

	//Draw viewing angle lines
	double viewing_angle = 72 * PI/180; //deg to rad
	double view_ang_1 = PI/2 + viewing_angle/2;
	double view_ang_2 = PI/2 - viewing_angle/2;

	vector<Point> line_view_1_points = get_fov_line_points(dohyo, robot_position, view_ang_1);
	polylines(dohyo, line_view_1_points, false, Scalar(255, 0, 0), 2, 8);

	vector<Point> line_view_2_points = get_fov_line_points(dohyo, robot_position, view_ang_2);
	polylines(dohyo, line_view_2_points, false, Scalar(255, 0, 0), 2, 8);


	imshow("Dohyo", dohyo);
	waitKey(0);
}

vector<Point> get_fov_line_points(Mat image, Point2f point, double angle){
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

/*
Mat draw_dohyo_opponent(Mat input, Point2f coord, double slope){    
    Mat output = input.clone();

    Point2f opponent_point = edge_detection.translate_opponent_position(coord);

	Point2f corner[4];

	corner[0] = opponent_point;
	//circle(output, opponent_point, 20, (0, 0, 255), -1);

    double side_length_px = 100 / pix_to_mm;

	double b = opponent_point.y - slope * opponent_point.x;
    double x = opponent_point.x + side_length_px / sqrt(1 + pow(slope,2));
    double y = x * slope + b;

    corner[1] = Point(x,y);
    //circle(frame, Point(x,y), 20, (0, 0, 255), -1);

    slope = -1/slope;
    b = opponent_point.y - slope * opponent_point.x;
	x = opponent_point.x - side_length_px / sqrt(1 + pow(slope,2));
    y = x * slope + b;

    corner[2] = Point(x,y);
    //circle(frame, Point(x,y), 20, (0, 0, 255), -1);

    x = (corner[1].x - corner[0].x) + corner[2].x;
    y = (corner[1].y - corner[0].y) + corner[2].y;

    corner[3] = Point(x,y);
    //circle(frame, Point(x,y), 20, (0, 0, 255), -1);

    line( output, corner[0], corner[1], Scalar(255), 5, 1 );
    line( output, corner[0], corner[2], Scalar(255), 5, 1 );
    line( output, corner[3], corner[1], Scalar(255), 5, 1 );
    line( output, corner[3], corner[2], Scalar(255), 5, 1 );

    return output;
}

Mat draw_lines(Mat frame){
	Point tangent_point = edge_detection.get_tangent_point();
    Mat line_tangent = edge_detection.get_line_tangent();
    Mat line_normal  = edge_detection.get_line_normal(); 

	vector<Point> line_tangent_points = this->get_line_points(frame, line_tangent);
	polylines(frame, line_tangent_points, false, Scalar(255, 0, 0), 2, 8);

	vector<Point> line_normal_points = this->get_line_points(frame, line_normal);
	polylines(frame, line_normal_points, false, Scalar(255, 0, 0), 2, 8);

    circle(frame, tangent_point, 5, (0, 0, 255), -1);

	return frame;
}

Mat draw_lines_opponent(Mat frame, Point2f coord, double slope){
	Point2f corner[4];

	corner[0] = this->homography_calc_inverse(coord);
	//circle(frame, this->homography_calc_inverse(coord), 5, (0, 0, 255), -1);

    double side_length_px = 100 / pix_to_mm;

	double b = coord.y - slope * coord.x;
    double x = coord.x + side_length_px / sqrt(1 + pow(slope,2));
    double y = x * slope + b;

    corner[1] = this->homography_calc_inverse(Point(x,y));
    //circle(frame, this->homography_calc_inverse(Point(x,y)), 5, (0, 0, 255), -1);

    slope = -1/slope;
    b = coord.y - slope * coord.x;
	x = coord.x - side_length_px / sqrt(1 + pow(slope,2));
    y = x * slope + b;

    corner[2] = this->homography_calc_inverse(Point(x,y));
    //circle(frame, this->homography_calc_inverse(Point(x,y)), 5, (0, 0, 255), -1);

    line( frame, corner[0], corner[1], Scalar(255), 2, 1 );
    line( frame, corner[0], corner[2], Scalar(255), 2, 1 );

	return frame;
}
*/