#include "../inc/image_capture.hpp"
#include "image_processing.cpp"

image_capture::image_capture(){
    // open the default camera using default API cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    //int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    int apiID = cv::CAP_GSTREAMER;
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    cap.set(CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CAP_PROP_FRAME_HEIGHT,240);
    // check if we succeeded
    if (!cap.isOpened()) {
        cout << "ERROR: Unable to open camera." << endl;
    }
    else{
        cout << "INFO: Camera opened successfully." << endl;
    }

    // Prepare folder for saving images
    path_to_saves = "../samples/saved/";
    img_extension = ".jpg";
    img_number = 0;
}

image_capture::~image_capture(){
	// the camera will be deinitialized automatically in VideoCapture destructor
}

/*
 *	Grab images from camera.
 */
Mat image_capture::take_image(){
	uint i = 0;
	uint max_tries = 10;

	while(i < max_tries){
	    // wait for a new frame from camera and store it into 'frame'
	    cap.read(frame);
	    // check if we succeeded
	    if (frame.empty()) {
	        cout << "ERROR: blank frame grabbed." << endl;
	    }
	    else{
	    	break;
	    }
	    i++;
	}

	this->save_image();
    return frame;
}

void image_capture::save_image(){
	img_number++;

	string path = path_to_saves + to_string(img_number) + img_extension;
	imwrite(path, frame);

	cout << path << endl;
}
