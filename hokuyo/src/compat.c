#include <time.h>
#include <sys/time.h>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>
#include <string.h>

#include "compat.h"

long timeMillis() {
	struct timeval tv;       
	if(gettimeofday(&tv, NULL) != 0) return 0;
	return (unsigned long)((tv.tv_sec * 1000ul) + (tv.tv_usec / 1000ul));        
}

char serial_read(int serial) {
	char data;
	int nbr = 0;
	do {
		nbr = read (serial, &data, 1);
	} while (nbr <= 0);
	data &= 0xFF;
	return data;
}

int nonblocking_read(int serial, char *data) {
	set_blocking(serial, 0);
	int n = read (serial, data, 1);
	set_blocking(serial, 1);
	return n;
}

int set_interface_attribs (int fd, int speed, int parity) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	tty.c_iflag &= ~(IGNBRK | INLCR | IGNCR | ICRNL);         // ignore break signal
	tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls, enable reading
	tty.c_cflag &= ~(PARENB);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0);
}
