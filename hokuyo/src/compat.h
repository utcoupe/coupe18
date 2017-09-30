#ifndef COMPAT_H
#define COMPAT_H

typedef enum bool 
{ 
	true = 1, false = 0 
} bool;

long timeMillis();

char serial_read(int serial);
int nonblocking_read(int serial, char *data);
int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);

#endif
