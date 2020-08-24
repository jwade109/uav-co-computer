/*
 * The values for speed are B115200, B230400, B9600, B19200, B38400, B57600, B1200, B2400, B4800, etc.
 * The values for parity are 0 (meaning no parity), PARENB|PARODD (enable parity and use odd), PARENB (enable parity and use even),
 * PARENB|PARODD|CMSPAR (mark parity), and PARENB|CMSPAR (space parity).
 * "Blocking" sets whether a read() on the port waits for the specified number of characters to arrive.
 * Setting no blocking means that a read() returns however many characters are available without waiting for more, up to the buffer limit.
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <fstream>
#include <stdint.h>
#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
using namespace std;

int fd;
uint16_t len;
uint8_t buf[BUFFER_LENGTH];
int set_interface_attribs (int fd, int speed, int parity)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
  {
    printf ("error %d from tcgetattr", errno);
    return -1;
  }
  
  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);
  
  //inout flags
  tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
  //output flags
  tty.c_oflag = 0;
  //line processing
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  //char processing
  tty.c_cflag &= ~(CSIZE | PARENB);
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~HUPCL;   // disable hang-up-on-close to avoid reset
  // One input byte is enough to return from read()  Inter-character timer off
  tty.c_cc[VMIN]  = 1;
  tty.c_cc[VTIME] = 0;
  
  
  if (tcsetattr (fd, TCSANOW, &tty) != 0)
  {
    printf ("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}

void set_blocking (int fd, int should_block)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
  {
    printf ("error %d from tggetattr", errno);
    return;
  }
  
  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
  
  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    printf ("error %d setting term attributes", errno);
}





