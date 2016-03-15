/*
 *  Message Client Process
 *
 *  This program is for JR3/Nitta Force moment sensor.
 *  Copyright(C) by Waseda University, Nitta Corporation. 2002.
 *
 * Modified by Kei Okada, 2014
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/mman.h>

#define Jr3DmAddrMask	0x6000
#define PAGE_SIZE	0x1000
unsigned long MappedAddress;

#define SENSOR0		0
#define SENSOR1		0x80000
#define SENSOR2		0x100000
#define SENSOR3		0x180000

typedef struct
{
  uint16_t msg_no;
  char msg_data[255];
} client_msg_t;

int
main (int argc, char **argv)
{
  int fd;
  int c;
  client_msg_t msg;
  int ret;
  int num;
  char msg_reply[255];
  int i;

  printf ("\x1b[2J");
  printf ("\x1b[0;0H");

  num = 4;

  /* Process any command line arguments */
  while ((c = getopt (argc, argv, "n:")) != -1)
    {
      if (c == 'n')
	{
	  num = strtol (optarg, 0, 0);
	}
    }

  /* Open a connection to the server (fd == coid) */
  fd = open ("/dev/jr3q", O_RDWR);
  if (fd == -1)
    {
      fprintf (stderr, "Unable to open server connection: %s\n",
	       strerror (errno));
      return EXIT_FAILURE;
    }

  /* Clear the memory for the msg and the reply */
  memset (&msg, 0, sizeof (msg));
  memset (&msg_reply, 0, sizeof (msg_reply));

  /* Setup the message data to send to the server */
  msg.msg_no = _IO_MAX + num;
  snprintf (msg.msg_data, 254, "client %d requesting reply.", getpid ());

  printf ("client: msg_no: _IO_MAX + %d\n", num);
  fflush (stdout);

  i = 0;
loop:
  /* Send the data to the server and get a reply */
  msg.msg_no = _IO_MAX + num;
  ret = MsgSend (fd, &msg, sizeof (msg), msg_reply, 255);
  if (ret == -1)
    {
      fprintf (stderr, "Unable to MsgSend() to server: %s\n",
	       strerror (errno));
      return EXIT_FAILURE;
    }
  if (num == 1)
    {
      float tmp[12];
      memcpy (tmp, msg_reply, sizeof (float) * 12);
      printf ("client: msg_reply: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n",
	      tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
      printf ("                   %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n",
	      tmp[6], tmp[7], tmp[8], tmp[9], tmp[10], tmp[11]);
    }
  else
    {
      printf ("client: msg_reply:\n%s\n", msg_reply);
    }

  if (num > 1)
    num--;

  usleep (100000);
  if ((i % 20) == 0)
    {
      num = 4;
    }


  if (i++ < 100000)
    goto loop;

  close (fd);

  return EXIT_SUCCESS;
}
