//#define DEBUG
#define BENCHMARK

#include <opencv2/highgui.hpp>
#include <iostream>
#include <iomanip>

#include "image_processing.cpp"
#include "sumo_main.cpp"

using namespace cv;
using namespace std;

/*------------------------------------------------*/
#ifdef BENCHMARK
#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration;

void benchmark();
#endif
/*------------------------------------------------*/
#ifdef DEBUG
#include "image_core.cpp"

image_core img_core(2,2);
#endif
/*------------------------------------------------*/

sumo_main sumo;

void run(String);

uint num_of_images = 35;

int main(){

	#ifndef BENCHMARK
	//String path_to_image = "../samples/Opponent/1.jpg";
	String path_to_image = "../samples/1.jpg";
	run(path_to_image);
	#else
	benchmark();
	#endif

 	waitKey(0);

    return 0;
}


void run(String path_to_image){
	//Take picture

	Mat image = imread(path_to_image, IMREAD_COLOR);
    if( image.empty() ){
        cout << "Could not open or find the image" << endl;
    }
    imshow("afs", image);
    waitKey(0);

    sumo.run(image);

	//Send to STM32 through uart
	//cout << robot_position << endl;
}


#ifdef BENCHMARK
void benchmark(){
	cout << "Benchmarks: " << endl;
	String path_to_image;
	Mat image;
	Point2f robot_position;

	auto t_start  = high_resolution_clock::now();
	auto t_end  = high_resolution_clock::now();
	duration<double, std::milli> ms_double[num_of_images];


	auto t_start_total  = high_resolution_clock::now();

	for(uint i = 1; i <= num_of_images; i++){
		t_start = high_resolution_clock::now();

		path_to_image = "../samples/" + to_string(i) + ".jpg";

		run(path_to_image);

		t_end = high_resolution_clock::now();

		/* Getting number of milliseconds as a double. */
		ms_double[i-1] = t_end - t_start;	
	}


	auto t_end_total  = high_resolution_clock::now();
	duration<double, std::milli> ms_double_total = t_end_total - t_start_total;

	double avg_execution = ms_double_total.count()/num_of_images;

	double avg_deviation = 0;
	double max_deviation = 0;

	double temp_deviation = 0;
	for (uint i = 0; i < num_of_images; i++){
		temp_deviation = (ms_double[i].count()*1.0) - avg_execution;
		avg_deviation += abs(temp_deviation);

		if (abs(max_deviation) < abs(temp_deviation)){
			max_deviation = temp_deviation;
		}
	}
	avg_deviation /= num_of_images;

    cout << fixed;
	cout << setprecision(3);
  	cout << "Execution time total:         " << ms_double_total.count() << " ms" << endl;
  	cout << "Execution time avg per image: " << avg_execution << "   ms" << endl;
  	cout << "Average deviation:            " << abs(avg_deviation) << "   ms" << endl;
  	cout << "Maximum deviation:            " << max_deviation << "   ms"<< endl;

}
#endif