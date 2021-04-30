#include "../inc/image_calibration.hpp"

image_calibration::image_calibration(){
	settings_file_path = "../calibration/calib.yaml";
}

// https://learnopencv.com/camera-calibration-using-opencv/

/*
 * Internal Parameters
 * Camera/lens system. E.g. focal length, optical center, and radial distortion coefficients of the lens.
 */

/*
 * External Parameters
 * This refers to the orientation (rotation and translation) of the camera with respect to some world coordinate system.
 */

bool image_calibration::get_settings(){
    //this->remove_calib_file();

    if (check_file_exists(settings_file_path)){
        cout << "Camera configuration file exists, reading parameters" << endl;
        this->read_parameters();
    }
    else if (check_images_exist()){
        cout << "Camera configuration does not exist, creating parameters" << endl;
        this->calibrate(); 
    }
    else{
        cout << "Taking images for camera calibration" << endl;
        this->take_calibration_images(); 
        this->calibrate(); 
    }

    this->find_homography_matrix();

    return true;
}

void image_calibration::take_calibration_images(){

}

void image_calibration::calibrate(){
    // Creating vector to store vectors of 3D points for each checkerboard image
    vector<vector<Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    vector<vector<Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    vector<Point3f> objp;
    for(int i{0}; i < CHECKERBOARD[1]; i++){
        for(int j{0}; j < CHECKERBOARD[0]; j++)
            objp.push_back(Point3f(j,i,0));
    }


    // Extracting path of individual image stored in a given directory
    vector<String> images;
    // Path of the folder containing checkerboard images
    string path = "../calibration/*.jpg";

    glob(path, images);

    Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners 
    vector<Point2f> corner_pts;
    bool success;

    // Looping over all the images in the directory
    for(int i{0}; i<images.size(); i++){
        frame = imread(images[i]);
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        success = findChessboardCorners(gray, CHECKERBOARD_SIZE, corner_pts, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

        //this->display_calib_images(frame, corner_pts, success);

        //If desired number of corner are detected, we refine the pixel coordinates and display them on the images of checker board
        if(success){
            TermCriteria criteria(TermCriteria::MAX_ITER | TermCriteria::EPS , 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cornerSubPix(gray,corner_pts, Size(11,11), Size(-1,-1), criteria);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }
    }

    /*
    * Performing camera calibration by passing the value of known 3D points (objpoints)
    * and corresponding pixel coordinates of the detected corners (imgpoints)
    */
    calibrateCamera(objpoints, imgpoints, Size(gray.rows,gray.cols), this->cameraMatrix, this->distCoeffs, this->R, this->T);

    cv::Size s = frame.size();

    this->newCameraMatrix, this->roi = getOptimalNewCameraMatrix(this->cameraMatrix, this->distCoeffs, s, 1, s);

    Mat ident = Mat::eye(3,3,CV_32F);

    initUndistortRectifyMap(this->cameraMatrix, this->distCoeffs, ident, this->newCameraMatrix, s, 5, this->mapx, this->mapy);

    this->save_parameters();
}

//Replacement for SolvePnP()
void image_calibration::find_homography_matrix(){
    //Get image
    string path = "../calibration/homography.jpg";
    Mat frame = imread(path);


    /*___________________Calculate Real World Points___________________*/

    //Calculate object points
    vector<Point3f> objectPoints;
    for( int i = 0; i < CHECKERBOARD[1]; i++ )
        for( int j = 0; j < CHECKERBOARD[0]; j++ )
            objectPoints.push_back(Point3f(float(j*CHECKERBOARD_SQUARE_SIZE),
                                      float(i*CHECKERBOARD_SQUARE_SIZE), 0));

    //The coordinate Z=0 must be removed for the homography estimation part
    vector<Point2f> objectPointsPlanar;
    for (size_t i = 0; i < objectPoints.size(); i++){
        objectPointsPlanar.push_back(Point2f(objectPoints[i].x, objectPoints[i].y));
    }


    /*___________________Calculate Image Points___________________*/

    //Detect chessboard pattern
    vector<Point2f> corners;
    bool found = findChessboardCorners(frame, CHECKERBOARD_SIZE, corners);

    //Undistort points using the intrinsic camera parameters
    vector<Point2f> imagePoints;
    undistortPoints(corners, imagePoints, this->cameraMatrix, this->distCoeffs);


    /*___________________Calculate Homography___________________*/

    Mat H = findHomography(objectPointsPlanar, imagePoints);
    cout << "\nH:\n" << H << endl;


    /*___________________Calculate Distance to normal___________________*/

    Mat rvec, tvec;
    solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
    cout << "\nrvec:\n" << rvec << endl;
    cout << "\ntvec:\n" << tvec << endl;

    Mat R1;
    Rodrigues(rvec, R1);
    
    Mat normal = R1*(Mat_<double>(3,1) << 0, 0, 1);
    cout << "\nnormal:\n" << normal << endl;

    Mat origin(3, 1, CV_64F, Scalar(0));
    Mat origin1 = R1*origin + tvec;
    cout << "\norigin:\n" << origin1 << endl;

    //The distance d can be computed as the dot product between the plane normal and a point on the plane
    double d = normal.dot(origin1);

    cout << "\nd: " << d << endl;
}

void image_calibration::save_parameters(){
    cv::FileStorage file(settings_file_path, cv::FileStorage::WRITE);
    file << "cameraMatrix" << this->cameraMatrix;
    file << "distCoeffs" << this->distCoeffs;
    file << "R" << this->R;
    file << "T" << this->T;
    file << "newCameraMatrix" << this->newCameraMatrix;
    file << "roi" << this->roi;
    file << "mapx" << this->mapx;
    file << "mapy" << this->mapy;
    file.release();

    //this->print_parameters();
}

void image_calibration::read_parameters(){
    cv::FileStorage file(settings_file_path, cv::FileStorage::READ);
    file["cameraMatrix"] >> this->cameraMatrix;
    file["distCoeffs"] >> this->distCoeffs;
    file["R"] >> this->R;
    file["T"] >> this->T;
    file["newCameraMatrix"] >> this->newCameraMatrix;
    file["roi"] >> this->roi;
    file["mapx"] >> this->mapx;
    file["mapy"] >> this->mapy;
    file.release();

    //this->print_parameters();
}

void image_calibration::print_parameters(){
    cout << "cameraMatrix : " << this->cameraMatrix << endl;
    cout << "distCoeffs : " << this->distCoeffs << endl;
    cout << "Rotation vector : " << this->R << endl;
    cout << "Translation vector : " << this->T << endl;
    cout << "newCameraMatrix : " << this->newCameraMatrix << endl;
    cout << "roi : " << this->roi << endl;
    //cout << "mapx : " << this->mapx << endl;
    //cout << "mapy : " << this->mapy << endl;
}

inline bool image_calibration::check_file_exists (const std::string& name) {
    struct stat buffer;   
    return (stat (name.c_str(), &buffer) == 0); 
}

inline bool image_calibration::check_images_exist(){
    //TODO check if enough images are taken to create configuration parameters

    return true;
}

void image_calibration::remove_calib_file(){
    if( remove( settings_file_path.c_str() ) != 0 )
        cout << "Error deleting camera calibration file" << endl;
    else
        cout << "Camera calibration file successfully deleted" << endl;
}

void image_calibration::remove_calib_images(){

}

void image_calibration::display_calib_images(Mat frame, vector<Point2f> corner_pts, bool success){
    // Displaying the detected corner points on the checker board
    drawChessboardCorners(frame, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
    
    cv::imshow("Image",frame);
    cv::waitKey(0);
    
    cv::destroyAllWindows();
}