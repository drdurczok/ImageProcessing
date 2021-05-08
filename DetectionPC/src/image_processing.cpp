#include "../inc/image_processing.hpp"

image_processing::image_processing(){
    calibration_file_path = "../calibration/calib.yaml";
	settings_file_path = "../calibration/settings.yaml";

	HCenter = Point2f(0,0);
	radius = 0;

	this->read_camera_parameters(calibration_file_path);
	this->read_camera_parameters(settings_file_path);

	this->calculate_camera_coordinates();
}

Point2f image_processing::getCircleCenter(){
	return HCenter;
}

float image_processing::getCircleRadius(){
	return radius;
}

Mat image_processing::filter(Mat input){
	if (input.empty()){
		cout << "Error: missing image\n\n" << endl;
	}

	//Convert image
	Mat output, kernel;
	cvtColor(input, output, COLOR_RGB2GRAY);

	//output = this->undistort(output);

	/*
	 *	Make binary image
	 *
	 *	threshold(src, dst, min, max)
	 */
	threshold(output, output, 210, 255, THRESH_BINARY);

	/*	
	 *  Closing morphology, removes small holes
	 *	
	 *	Closing = erode(dilute(src, dst, kernel))
	 */
	kernel = getStructuringElement(MORPH_RECT, Size(5,5));
	morphologyEx(output, output, MORPH_CLOSE, kernel);

	/*	
	 *  Opening morphology, removes small objects
	 *	
	 *	Opening = dilate(erode(src, dst, kernel))
	 */
	kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
	morphologyEx(output, output, MORPH_OPEN, kernel);

	/*
	 *	Blurring, smooth the edges
	 *
	 *	GaussianBlur(src, dst, kernel size, sigma)
	 *		sigma - how much image blur
	 */
	GaussianBlur(output, output, Size(3,3), 1);

	return output;
}

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

//_________________________________________________________________
Mat image_processing::calculate_circle_dimensions(Mat img){
	Mat mask = this->find_edge(img, FLOOR_PIXELS);
	ellipse_detection(mask);

	return mask;
}

Mat image_processing::ellipse_detection(Mat mask){
	Point2f center;
	float   radius;

    Mat output = Mat::zeros(mask.rows, mask.cols, CV_8UC1);

	vector<Point2f> edgePositions;
	edgePositions = getCirclePoints(mask);

	// create distance transform to efficiently evaluate distance to nearest edge
	Mat dt;
	distanceTransform(255-mask, dt, DIST_L1, 3);

    // create circle from 3 points:
    getCircle(edgePositions[0],edgePositions[1],edgePositions[2],center,radius);

    float minCirclePercentage = 0.0015f;

    float cPerc = verifyCircle(dt, center, radius);

    if(cPerc >= minCirclePercentage && radius > 6000){
        //cout << "circle with center: " << center << " radius: " << radius << " inliers: " <<  cPerc*100.0f  << "%" << endl;
        circle(output, center, radius, Scalar(255,0,0),1);
	}

	this->HCenter = this->transform_to_homography_coord(center);
	this->radius = radius;

	return output;
}

/*
 *  Verifies the circle by counting number of inliers in arc.
 */
float image_processing::verifyCircle(Mat dt, Point2f center, float radius){
	unsigned int counter = 0;
	unsigned int inlier = 0;
	float minInlierDist = 2.0f;
	float maxInlierDistMax = 100.0f;
	float maxInlierDist = radius/25.0f;
	if(maxInlierDist<minInlierDist) maxInlierDist = minInlierDist;
	if(maxInlierDist>maxInlierDistMax) maxInlierDist = maxInlierDistMax;

	// choose samples along the circle and count inlier percentage
	for(float t =0; t<2*3.14159265359f; t+= 0.05f){
		counter++;
		float cX = radius*cos(t) + center.x;
		float cY = radius*sin(t) + center.y;

		if(cX < dt.cols)
		if(cX >= 0)
		if(cY < dt.rows)
		if(cY >= 0)
		if(dt.at<float>(cY,cX) < maxInlierDist){
			inlier++;
		}
	}

	return (float)inlier/float(counter);
}

/*
 *	Calculates center of circle and radius based off 3-points
 */
inline void image_processing::getCircle(Point2f& p1, Point2f& p2, Point2f& p3, Point2f& center, float& radius){
	cout << "Point 1: " << p1 << "\nPoint 2: " << p2 << "\nPoint 3: " << p3 << endl;
	float x1 = p1.x;
	float x2 = p2.x;
	float x3 = p3.x;

	float y1 = p1.y;
	float y2 = p2.y;
	float y3 = p3.y;

	center.x = (x1*x1+y1*y1)*(y2-y3) + (x2*x2+y2*y2)*(y3-y1) + (x3*x3+y3*y3)*(y1-y2);
	center.x /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

	center.y = (x1*x1 + y1*y1)*(x3-x2) + (x2*x2+y2*y2)*(x1-x3) + (x3*x3 + y3*y3)*(x2-x1);
	center.y /= ( 2*(x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2) );

	radius = sqrt((center.x-x1)*(center.x-x1) + (center.y-y1)*(center.y-y1));
}

/*
 *	Get three points of arc that will be used to calculate circle
 */
std::vector<cv::Point2f> image_processing::getCirclePoints(Mat binaryImage){
	std::vector<cv::Point2f> pointPositions;

	/*
	 * Use of pointers is fastest, however requires proper range calculations to avoid overflow
	 * "at" does a type check to determine range of type, but is much more resource intensive
	 * Using row pointers avoids the overflow problem with pointers but is much faster than "at"
	 */
	for(unsigned int y = 0; y < binaryImage.rows; ++y){
		unsigned char* rowPtr = binaryImage.ptr<unsigned char>(y);
		
		for(unsigned int x = 0; x < binaryImage.cols; ++x){
			if(rowPtr[x] > 0) pointPositions.push_back(cv::Point2i(x,y));
			//if(binaryImage.at<unsigned char>(y,x) > 0) pointPositions.push_back(Point2f(x,y));
		}
	}

	sort(pointPositions.begin(), pointPositions.end(), [](const Point2f &a, const Point2f &b) {
	    return (a.x < b.x);
	});

	cout << pointPositions << endl;

	//TODO improve decision method of choosing arc points
	//Method 1: choose three random points
    //unsigned int idx1 = rand()%pointPositions.size();
    //unsigned int idx2 = rand()%pointPositions.size();
    //unsigned int idx3 = rand()%pointPositions.size();

   	//Method 2: choose from three distinct segments of curve
   	uint size = pointPositions.size()/6;
    unsigned int idx1 = (rand()%size);
    unsigned int idx2 = (rand()%size) + 2*size;
    unsigned int idx3 = (rand()%size) + 4*size;



   	std::vector<cv::Point2f> edgePositions;
   	edgePositions.push_back(pointPositions[idx1]);
   	edgePositions.push_back(pointPositions[idx2]);
   	edgePositions.push_back(pointPositions[idx3]);

	return edgePositions;
}

Point2f image_processing::transform_to_homography_coord(Point2f center){
	return center;
}


//_________________________________________________________________
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

Mat image_processing::undistort(Mat img){
	read_camera_parameters(calibration_file_path);

	Mat output;
	remap(img, output, this->mapx, this->mapy, INTER_LINEAR);

	return output;
}

//_________________________________________________________________

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
    warpPerspective(frame, homography_frame, this->homographyMatrixInv, frame.size());

    return homography_frame;
}

Mat image_processing::get_homography_origin_frame(Mat frame){
	uint CHECKERBOARD_SQUARE_SIZE = 25;
    drawFrameAxes(frame, this->cameraMatrix, this->distCoeffs, this->rvec, this->tvec, 2*CHECKERBOARD_SQUARE_SIZE);

    return frame;
}

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
	cout << "Drawing circle" << endl;
	circle(frame, pixel, 6, (0, 0, 255), -1);

	return frame;
}

double image_processing::get_pix_to_mm(){
	return this->pix_to_mm;
}
