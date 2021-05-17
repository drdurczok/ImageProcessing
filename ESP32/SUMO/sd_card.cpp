#include "sd_card.h"

int pictureNumber = 0;

void init_sd(){
  if(!SD_MMC.begin()){
      Serial.println("Card Mount Failed, rebooting...");
      ESP.restart();
      return;
  }
  
  fs::FS &tfs = SD_MMC;
  uint8_t cardType = SD_MMC.cardType();
  switch (cardType) {
    case CARD_NONE : Serial.println("No SD Card type found");
    break;
    case CARD_MMC : Serial.println("MMC Card detected…");
    break;
    case CARD_SD : Serial.println("SD Card detected…");
    break;
    case CARD_SDHC : Serial.println("SDHC Card detected…");
    break;
    default : Serial.println("Default card detected ???");
    break;
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
}

void test_sd(){
  
}

//Read a file from SD card
void readFile(String path){
  File file = SD_MMC.open(path, FILE_READ);
 
  if (!file) {
    Serial.println("Opening file to read failed");
    return;
  }
 
  Serial.println("File Content:");
 
  while (file.available()) {
    Serial.write(file.read());
  }
 
  file.close();
}

//Append to the end of file in SD card
void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);
  
  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void writeFrame(camera_fb_t * frame){  
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;
 
  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) +".jpg";
 
  fs::FS &fs = SD_MMC; 
  Serial.printf("Picture file name: %s\n", path.c_str());
   
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file in writing mode");
  } 
  else {
    file.write(frame->buf, frame->len); // payload (image), payload length
    //Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();
  esp_camera_fb_return(frame); 
}


camera_fb_t * readFrame(String path){
  camera_fb_t * frame_read;

  fs::FS &fs = SD_MMC; 
  File file = fs.open(path.c_str(), FILE_READ);
  if(!file){
    Serial.println("Failed to open file in reading mode");
  } 
  else {
    file.read(frame_read->buf, frame_read->len); // payload (image), payload length
  }
  file.close();

  return frame_read;
}
