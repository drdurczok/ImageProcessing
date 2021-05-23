#include "../inc/communications.hpp"

communications::communications(){
    this->serial_port = open("/dev/ttyUSB0", O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        CERR("Error " + to_string(errno) + " from open, " + strerror(errno));
    }

}

void communications::send_uart(){
	unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
	write(this->serial_port, msg, sizeof(msg));
}