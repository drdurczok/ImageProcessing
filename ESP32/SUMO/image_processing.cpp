#include "image_processing.h"
#include "Arduino.h"

double homographyInv[3][3];

void get_config(){

}

void print_image_shape(){
    // print shape of image and total length (=heigth*width)
    Serial.print("Width: ");
    Serial.print(frame->width);
    Serial.print("\tHeigth: ");
    Serial.print(frame->height);
    Serial.print("\tLength: ");
    Serial.println(frame->len);
}

void homography(){
    int len = frame->len;
    int buf_position;
    uint16_t pixel;

    // Access all pixels in frame
    for (int h=0; h < HEIGHT; h++){
        for (int w=0; w < WIDTH; w++){
            buf_position = h*(len/HEIGHT)+w;

            pixel = frame->buf[buf_position];

            //Multiply by homography matrix
        }
    }
}
