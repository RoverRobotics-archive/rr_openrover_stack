// A C library for accessing an OpenRover Robot

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>

#include "openrover/openrover_lib.hpp"

size_t write_openrover_speed_command(unsigned char address, char channel, int speed, unsigned char* buffer)
{
	unsigned char data[9] = { 0 };
	size_t length = 0;
	data[length] = (unsigned char)channel;
	length++;
	data[length] = 32;  // move flags
	length++;
	data[length] = 2;  // Speed
	length++;
	length += bitpackNumber(&data[length], speed);
	return write_kangaroo_command(address, 36, data, length, buffer);
}



