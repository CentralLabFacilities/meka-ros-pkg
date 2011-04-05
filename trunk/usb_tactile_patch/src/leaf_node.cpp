#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "ros/ros.h"
#include "usb_tactile_patch/UsbTactilePatch.h"
#include <sstream>

#define BUF_SIZE 200
#define MSG_SIZE 24

/*
  * 'open_port()' - Open serial port 1.
  *
  * Returns the file descriptor on success or -1 on error.
  */

int
open_port(void)
{
  int fd; /* File descriptor for the port */


  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  {
    /*
    * Could not open the port.
    */

    perror("open_port: Unable to open /dev/ttyS0 - ");
  }
  else
    fcntl(fd, F_SETFL, 0);

  return (fd);
}


int main(int argc, char **argv)
{
  int fd = open_port();
  
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<usb_tactile_patch::UsbTactilePatch>("chatter", 1000);
  
  
  struct termios options;

    /*
     * Get the current options for the port...
     */

    tcgetattr(fd, &options);

    /*
     * Set the baud rates to 19200...
     */

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    /*
     * Enable the receiver and set local mode...
     */

    options.c_cflag |= (CLOCAL | CREAD);
    /*  8N1 */
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    /*
     * Set the new options for the port...
     */        
    tcsetattr(fd, TCSANOW, &options);
    
    int tactiles[12];
    unsigned char buf[BUF_SIZE];
    int cnt = 0;
    int buf_start = 0;
    int buf_filled;    
    int buf_reader = 0;
    int bytes_read;
    
    while (1)
    {
      bytes_read = read(fd, &buf[buf_start], BUF_SIZE);
      if (cnt != -1)
      {
	buf_filled = bytes_read + buf_start;
	if (buf_filled >= MSG_SIZE * 2) // make sure we have at least one msg
	{
	  // parse buf
	  while (buf_reader < buf_filled)
	  {	    
	    if (buf[buf_reader] == 115)  // starting
	    {
	      if (buf_reader < buf_filled - MSG_SIZE) // parse the msg we should have a whole one
	      {
		if (buf[buf_reader+25] == 102) // make sure we have a msg and not a false pos
		{
		  for (int i = 0; i < 12; i++) // parse the values...
		  {
		      unsigned int value;
		      unsigned char * ptr;
		      ptr = (unsigned char*)(&value);
		      ptr[0] = buf[buf_reader+i*2];
		      ptr[1] = buf[buf_reader+i*2+1];
		      tactiles[i] = value;
		  }
		  buf_reader += MSG_SIZE - 1;
		}
		buf_reader++;
	      } else { // to close to end of buf copy to front and get it next time..
		int j = 0;
		for (int i = buf_reader; buf_reader < buf_filled; i++)
		{
		    buf[j] = buf[i];
		    j++;
		}
		buf_reader = buf_filled - MSG_SIZE;
	      }
	    }	    
	  }	  
	  buf_start = 0;
	} else {
	  buf_start += cnt;
	}
      } else
	printf("error reading usb\n");
    }
    
    //chatter_pub.publish(msg);

    
  close(fd);
 
  return 0;
}