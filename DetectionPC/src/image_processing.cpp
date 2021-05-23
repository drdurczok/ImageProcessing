#include "../inc/image_processing.hpp"

image_processing::image_processing(){
    calibration_file_path = "../calibration/calib.yaml";
	settings_file_path = "../calibration/settings.yaml";

	HCenter = Point2f(0,0);
	radius = 0;

	this->read_camera_parameters(calibration_file_path);
	this->read_camera_parameters(settings_file_path);


	this->read_file("../calibration/homography_map_x.txt", this->linearized_homogeneous_array[0]);
	this->read_file("../calibration/homography_map_y.txt", this->linearized_homogeneous_array[1]);

	this->calculate_camera_coordinates();
}

Point2f image_processing::getCircleCenter(){
	return HCenter;
}

float image_processing::getCircleRadius(){
	return radius;
}


void image_processing::prepare_image(Mat input, Mat & output){
	Debug("Filtering image.");
	if (input.empty()){
		CERR("Missing image.");
	}
	
	//Convert image
	cvtColor(input, output, COLOR_RGB2GRAY);

	//output = this->undistort(output);

	output = this->get_homography_frame(output);
}

/*______________________EDGE DETECTION___________________________*/

Mat image_processing::find_edge(Mat input, edge_filter_methods selection){	
	Mat output;

	switch (selection){
	    case SOBEL:
	    	output = this->sobelEdgeDetection(input);
	    	break;
	    case FLOOR_PIXELS:
	    	output = this->getFloorPixels(input);
	    	break;
	    case CEILING_PIXELS:
	    	output = this->getCeilingPixels(input);
	    	break;
	    default:
	    	output = this->sobelEdgeDetection(input);
	}

	return output;
}

Mat image_processing::sobelEdgeDetection(Mat input){
	Mat sobel, sobelx, sobely;

	/* 
	 * Sobel (input, output, ddepth, dx, dy, ksize=3, scale=1, delta=0, borderType=BORDER_DEFAULT)
	 *		ddepth - output image depth
	 *		dx, dy - order of the derivative
	 *		ksize  - size of the extended Sobel kernel; it must be 1, 3, 5, or 7
	 */

	Sobel(input, sobelx, CV_64F, 1, 0, 5);  // x
	Sobel(input, sobely, CV_64F, 0, 1, 5);  // y

	// Combine images together
	bitwise_and(sobelx, sobely, sobel);

	Mat output;
	sobel.convertTo(output, CV_8UC1, 1);

	return output;
}

/*
 * Copies only first pixel that appears at the bottom of the column
 */
Mat image_processing::getFloorPixels(Mat input){
    Mat bottom_pixels = Mat::zeros(input.rows, input.cols, CV_8UC1);
	
	for (uint i = 0; i < input.cols; i++){
		for(uint j = input.rows-1; j > 0; j--){
			if(input.at<uchar>(j,i) != 0){
				bottom_pixels.at<uchar>(j,i) = 255;
				break;
			}
		}
	}

	return bottom_pixels;
}

/*
 * Copies only first pixel that appears at the top of the column
 */
Mat image_processing::getCeilingPixels(Mat input){
    Mat top_pixels = Mat::zeros(input.rows, input.cols, CV_8UC1);
	
	for (uint i = 0; i < input.cols; i++){
		for(uint j = 0; j < input.rows; j++){
			if(input.at<uchar>(j,i) != 0){
				top_pixels.at<uchar>(j,i) = 255;
				break;
			}
		}
	}

	return top_pixels;
}

/*
 * Copies only first pixel that appears at the bottom of the column as point coord
 */
vector<Point2f> image_processing::getFloorPixels_Points(Mat input){
    vector<Point2f> bottom_pixels;
	
	for (uint i = 0; i < input.cols; i++){
		for(uint j = input.rows-1; j > 0; j--){
			if(input.at<uchar>(j,i) != 0){
				bottom_pixels.push_back(Point2f(i,j));
				break;
			}
		}
	}

	return bottom_pixels;
}


/*
 * Copies only first pixel that appears at the top of the column as point coord
 */
vector<Point2f> image_processing::getCeilingPixels_Points(Mat input){
    vector<Point2f> top_pixels;
	
	for (uint i = 0; i < input.cols; i++){
		for(uint j = 0; j < input.rows; j++){
			if(input.at<uchar>(j,i) != 0){
				top_pixels.push_back(Point2f(i,j));
				break;
			}
		}
	}

	return top_pixels;
}

/*______________________CAMERA___________________________________*/

void image_processing::read_camera_parameters(string path){
	cv::FileStorage file(path, cv::FileStorage::READ);
	if (path == calibration_file_path){
		file["cameraMatrix"] >> this->cameraMatrix;
        file["distCoeffs"] >> this->distCoeffs;
	    file["mapx"] >> this->mapx;
	    file["mapy"] >> this->mapy;
	}
	else if (path == settings_file_path){
		file["rvec"] >> this->rvec;
        file["tvec"] >> this->tvec;
        file["homographyMatrix"] >> this->homographyMatrix;
        file["homographyMatrixInv"] >> this->homographyMatrixInv;
        file["distanceToPlaneNormal"] >> this->distanceToPlaneNormal;
        file["pix_to_mm"] >> this->pix_to_mm;
	}
    file.release();
}

void image_processing::read_file(string path, int16_t array[]){
	ifstream File;
    
    File.open(path);
    if (!File) {
        CERR("Unable to open file");
        exit(1); // terminate with error
    }
    
    uint i = 0;
    int val;
    while (File >> val) {
        array[i] = val;
        i++;
    }
	
	File.close();
}

Mat image_processing::undistort(Mat img){
	read_camera_parameters(calibration_file_path);

	Mat output;
	remap(img, output, this->mapx, this->mapy, INTER_LINEAR);

	return output;
}

/*______________________HOMOGRAPHY_______________________________*/

Point2f image_processing::homography_calc(Point2f pixel){
    /*
     * Convert point in image frame to homography coordinates.
     *  1 pixel = 1 mm
     */
    //Convert to Mat
    double point_coord[] = {pixel.x, pixel.y, 1};
    Mat CPoint = Mat(3, 1, CV_64F, point_coord);

    //Calculate point in homography frame of reference
    Mat HPoint = this->homographyMatrixInv * CPoint;

    //Noramlize the point
    HPoint =  HPoint / HPoint.at<double>(2);

    //Convert back to Point2f
    return Point2f(HPoint.at<double>(0), HPoint.at<double>(1));
}

Mat image_processing::get_homography_frame(Mat frame){
	Mat homography_frame;

	//Mat large_frame = Mat::zeros(frame.size().height*2, frame.size().width*2, CV_8UC3);
	//frame.copyTo(large_frame(Rect((large_frame.cols - frame.cols)/2, (large_frame.rows - frame.rows)/2, frame.cols, frame.rows)));

    //warpPerspective(frame, homography_frame, this->homographyMatrixInv, frame.size());

    homography_frame = this->get_homography_frame_from_map(frame);

    return homography_frame;
}

Mat image_processing::get_homography_frame_from_map(Mat frame){
	// Read dimensions of new padded image
	//TODO
	//double image_size[2];
	//this->read_file("../calibration/homography_map_image_parameters.txt", image_size);


	// Warp image
	Mat homography_frame = Mat::zeros(this->rows, this->cols, CV_8U);

	int x, y;
	uint f_x, f_y;
	for (uint i=0; i < array_size; i++){
		x = this->linearized_homogeneous_array[0][i];
		y = this->linearized_homogeneous_array[1][i];

		if ((x > 0 && x < this->cols) && (y > 0 && y < this->rows)){
			f_x = i%this->cols;
			f_y = i/this->cols;
			homography_frame.at<uint8_t>(y,x) = frame.at<uint8_t>(f_y,f_x);
		}

	}

    return homography_frame;
}

Mat image_processing::get_homography_origin_frame(Mat frame){
	uint CHECKERBOARD_SQUARE_SIZE = 25;
    drawFrameAxes(frame, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec, 2*CHECKERBOARD_SQUARE_SIZE);

    return frame;
}

/*______________________DISTANCE_________________________________*/

void image_processing::calculate_camera_coordinates(){
	Mat R, cameraPosition;

    Rodrigues(this->rvec, R);

    cameraPosition = -R.t() * this->tvec;

    this->camera_coordinates = Point2f(cameraPosition.at<double>(0), cameraPosition.at<double>(1));
}

Point2f image_processing::get_camera_coordinates(){
	return this->camera_coordinates;
}


double image_processing::distance_camera_to_pixel(Point2f pixel){
	return sqrt(pow(pixel.x - this->camera_coordinates.x, 2) + pow(pixel.y - this->camera_coordinates.y, 2) * 1.0) * this->pix_to_mm; 
}

Mat image_processing::draw_point_on_frame(Mat frame, Point2f pixel){
	circle(frame, pixel, 6, (0, 0, 255), -1);

	return frame;
}

double image_processing::get_pix_to_mm(){
	return this->pix_to_mm;
}
