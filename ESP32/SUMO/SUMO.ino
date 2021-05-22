#include "esp_camera.h"
camera_fb_t * frame;

#include "image_capture.h"
#include "sd_card.h"

#include "image_processing.h"

unsigned long t_start;
unsigned long t_end;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  init_camera();
  init_sd();

  get_config();
}

void send_photo();

void loop() {
  delay(200);
  t_start = millis();
  
  take_photo();
  //processFrame();
  save_photo();

  //send_photo();
  
  t_end = millis();
  Serial.print("Execution time: ");
  Serial.print((t_end - t_start)/1000);
  Serial.println("s");
}

void send_photo(){
  
}
