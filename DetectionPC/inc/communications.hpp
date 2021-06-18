#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <filesystem>

using namespace std;

extern string architecture;

class communications {
  public:
    communications();
    ~communications();
    void send_uart(string);
    void read_uart();

  private:
    int serial_port;
    string path;
    ofstream file;

	char read_buf [256]; // Allocate memory for read buffer, set size according to your needs

	void save_to_file(string);


  /*______________________UART CONFIG___________________________*/
  int set_interface_attribs(int, int, int);
  void set_blocking(int, int);
};

#endif