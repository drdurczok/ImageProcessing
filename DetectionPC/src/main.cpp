//#define BENCHMARK
//#define CALIBRATION

#include <opencv2/highgui.hpp>
#include "sys.cpp"

#include "image_capture.cpp"
#include "sumo_main.cpp"

using namespace cv;
using namespace std;

int iterations = 0;

#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration;

void run(){
	image_capture camera;
	sumo_main sumo;

	auto t_start  = high_resolution_clock::now();
	duration<double, std::milli> ms_double;

	while(true){
		BREAK_LINE();

		Mat image = camera.take_image();
    	sumo.run(image);

    	ms_double = high_resolution_clock::now() - t_start;

    	//Finish after one minute
    	if(ms_double.count() > 60000){
    		break;
    	}
	}

	cout << "EXITING PROCESS" << endl;
}



/*------------------------------------------------*/
#ifdef BENCHMARK
#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration;

void benchmark();
#endif
/*------------------------------------------------*/
#ifdef CALIBRATION

void calibration();
#endif
/*------------------------------------------------*/

int main(){
	set_tags();

	#if   defined(BENCHMARK)
	benchmark();
	#elif defined(CALIBRATION)
	calibration();
	#else
	run();
	#endif

    return 0;
}



#ifdef BENCHMARK
image_capture camera;

Mat image;
uint num_of_runs = 1260;	//In the case of prepared samples, must be equal or less than the number of samples
String path_to_image;

void benchmark(){
	sumo_main sumo;

	// Take first image and give time for hardware to initialize
	//image = camera.take_image();
	//for (int j = 0; j<1000000; j++){}

	auto t_start  = high_resolution_clock::now();
	auto t_end  = high_resolution_clock::now();
	duration<double, std::milli> ms_double[num_of_runs];

	auto t_start_total  = high_resolution_clock::now();

	for(uint i = 1; i <= num_of_runs; i++){
		t_start = high_resolution_clock::now();

		/*_________START CORE__________*/
		// ADD METHODS TO BE BENCHMARKED
		//image = camera.take_image();
		path_to_image = "../samples/saved_ring/" + to_string(i) + ".jpg";

		image = imread(path_to_image, IMREAD_COLOR);
	    if( image.empty() ){
	        cout << "Could not open or find the image" << endl;
	    }
		sumo.run(image);
		/*_________END CORE__________*/

		t_end = high_resolution_clock::now();

		/* Getting number of milliseconds as a double. */
		ms_double[i-1] = t_end - t_start;	
	}


	auto t_end_total  = high_resolution_clock::now();
	duration<double, std::milli> ms_double_total = t_end_total - t_start_total;

	double avg_execution = ms_double_total.count()/num_of_runs;

	double avg_deviation = 0;
	double max_deviation = 0;

	double temp_deviation = 0;
	for (uint i = 0; i < num_of_runs; i++){
		temp_deviation = (ms_double[i].count()*1.0) - avg_execution;
		avg_deviation += abs(temp_deviation);

		if (abs(max_deviation) < abs(temp_deviation)){
			max_deviation = temp_deviation;
		}
	}
	avg_deviation /= num_of_runs;

    cout << fixed;
	cout << setprecision(3);
  	cout << "Execution time total:         " << ms_double_total.count() << " ms" << endl;
  	cout << "Execution time avg per image: " << avg_execution << "   ms" << endl;
  	cout << "Average deviation:            " << abs(avg_deviation) << "   ms" << endl;
  	cout << "Maximum deviation:            " << max_deviation << "   ms"<< endl;

}
#endif

void calibration(){
	image_capture camera;

	while(true){
		camera.take_image();
		camera.save_image();
	}
}