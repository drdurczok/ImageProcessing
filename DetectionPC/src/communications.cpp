#include "../inc/communications.hpp"

communications::communications(){
	string uart_path;
    if(architecture == "ARM"){
    	uart_path = "/dev/ttyS1";
	}
    else{
    	uart_path = "/dev/ttyUSB0";
    }
    Debug("Using UART device " + uart_path);
    cout << "Using UART device " << uart_path << endl;
   	this->serial_port = open(uart_path.c_str(), O_RDWR);

    // Check for errors
    if (this->serial_port < 0) {
        CERR("Error " + to_string(errno) + " from open, " + strerror(errno));
    }
    else{
    	set_interface_attribs(this->serial_port, B115200, 0);	// set speed to 115,200 bps, 8n1 (no parity)
		set_blocking(this->serial_port, 0);                		// set no blocking
	}

    this->path = "../results/UART_FEED.txt";
    this->file.open(this->path);
}

communications::~communications(){
	close(this->serial_port);
	this->file.close();
}

void communications::send_uart(string message){
	int len = message.length() + 1;
    char message_array[len];
    strcpy(message_array, message.c_str());
    cout << "Out: " << message << endl;
    UART_LOG(message_array);

    this->save_to_file("Out: ");
    this->save_to_file(message_array);
    this->save_to_file("\n");

	write(this->serial_port, message_array, len);

	usleep((len + 25) * 100); 
    
    this->read_uart();
}

void communications::read_uart(){
	// n is the number of bytes read. n may be 0 if no bytes were received, and can also be negative to signal an error.
	int n = read(this->serial_port, &this->read_buf, sizeof(this->read_buf));

	if (n > 0){
		string buff = "";
	    for (uint i = 0; i < 256; i++) {
	    	if (this->read_buf[i] == '\0'){
	    		break;
	    	}
	        buff = buff + this->read_buf[i];
	    }

	    cout << "In: " << buff << endl;

	    this->save_to_file("In:  ");
		this->save_to_file(buff);
		this->save_to_file("\n");
	}
}

void communications::save_to_file(string message_array){
	this->file << message_array;
	//this->file << "\n";
}





int communications::set_interface_attribs(int fd, int speed, int parity){
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0){
                CERR("error " + to_string(errno) + " from tcgetattr");
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0){
                CERR("error " + to_string(errno) + " from tcsetattr.");
                return -1;
        }
        return 0;
}

void communications::set_blocking(int fd, int should_block) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0){
                CERR("error " + to_string(errno) + " from tggetattr.");
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                CERR("error " + to_string(errno) + " setting term attributes.");
}