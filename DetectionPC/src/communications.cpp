#include "../inc/communications.hpp"

communications::communications(){
    if(architecture == "ARM"){
	    Debug("Changing UART device to /dev/ttyS1.");
	    this->serial_port = open("/dev/ttyS1", O_RDWR);
	}
    else{
    	this->serial_port = open("/dev/ttyUSB0", O_RDWR);
    }

    // Check for errors
    if (serial_port < 0) {
        CERR("Error " + to_string(errno) + " from open, " + strerror(errno));
    }

    this->path = "../results/UART_FEED.txt";
    this->file.open(this->path);
}

communications::~communications(){
	close(this->serial_port);
	this->file.close();
}

void communications::send_uart(string message){
	int len = message.length();
    char message_array[len + 1];
    strcpy(message_array, message.c_str());

    UART_LOG(message_array);
    this->save_to_file("Out: ", 7);
    this->save_to_file(message_array, len);

	write(this->serial_port, message_array, sizeof(message_array));
}

void communications::read_uart(){
	// n is the number of bytes read. n may be 0 if no bytes were received, and can also be negative to signal an error.
	int n = read(this->serial_port, &this->read_buf, sizeof(this->read_buf));

	this->save_to_file("In:  ", 7);
	this->save_to_file(this->read_buf, sizeof(this->read_buf));
}

void communications::save_to_file(string message_array, int len){
	for ( uint i = 0; i < len; i++){
		this->file << message_array[i];
	}
	this->file << "\n";
}