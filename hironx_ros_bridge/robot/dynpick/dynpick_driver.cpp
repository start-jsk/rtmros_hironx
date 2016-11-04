/*
 * This program is based on http://github.com/tork-a/dynpick_driver and http://github.com/start-jsk/rtmros_hironx/blob/indigo-devel/hironx_ros_bridge/robot/nitta/jr3_driver.cpp

  This driver is stored in this robot-specific package for not many reasons than they are slightly customized for the robot (as of Apr 2016 it takes 2 sensor inputs in a single cpp file. It also assumes the specific device file name). So if you can separate those as a standalone, generic package that'll be appreciated (please just let us know if you will at https://github.com/start-jsk/rtmros_hironx/issues).
*/
/*
 * This program is for Dynpick F/T sensor (developed from JR3 sensor).
 * Copyright(C) by Waseda University, Nitta Coropration. 2002.
 *
 * Copyright (c) 2016, TORK (Tokyo Opensource Robotics Kyokai Association)
 * All rights reserved.
 * # Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * #  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of TOKYO. nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 * # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/slog.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

unsigned short force_sensor_data_0[6];
unsigned short force_sensor_data_1[6];

/*
 * serial stuff
 */

#define cfsetspeed(term, baudrate)\
   cfsetispeed(term, baudrate);\
   cfsetospeed(term, baudrate);

int SetComAttr(int fdc) {
	int res = -1;
	if (fdc < 0) {
		char *pmesg = strerror(errno);
		fprintf(stderr, "failed to open : %s\n", pmesg);
		goto over;
	}

	struct termios term;
	res = tcgetattr(fdc, &term);

	if (res < 0) {
		char *pmesg = strerror(errno);
		fprintf(stderr, "failed to tcgetattr(): %s\n", pmesg);
		goto over;
	}
	cfmakeraw(&term);
	res = cfsetspeed(&term, 921600)
	;
	if (res < 0) {

		char *pmesg = strerror(errno);
		fprintf(stderr, "failed to cfsetspeed(): %s\n", pmesg);
		goto over;
	}

	 // settings for qnx
	 term.c_iflag |= IGNPAR;            // Ignore characters with parity errors
	 term.c_cflag |= (CLOCAL | CREAD);  // needed for QNX 6.3.2
	 term.c_cflag &= ~PARENB;           // disable parity check
	 term.c_cflag |= CS8;               // 8 data bit
	 term.c_cflag &= ~CSTOPB;           // 1 stop bit
	 //term.c_lflag = IEXTEN;
	 term.c_oflag = 0; ///added
	 term.c_lflag &= ~(ECHO | ECHOCTL | ECHONL);  // disable ECHO
	   
	 //
	 term.c_iflag &= ~INPCK;

	 // settings for dynpick
	   term.c_cc[VINTR]    = 0;     /* Ctrl-c */
	   term.c_cc[VQUIT]    = 0;     /* Ctrl-? */	   term.c_cc[VERASE]   = 0;     /* del */
	   term.c_cc[VKILL]    = 0;     /* @ */
	   term.c_cc[VEOF]     = 4;     /* Ctrl-d */
	   term.c_cc[VTIME]    = 0;
	   term.c_cc[VMIN]     = 0;
	   //term.c_cc[VSWTC]    = 0;     /* '?0' */
	   term.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
	   term.c_cc[VSTOP]    = 0;     /* Ctrl-s */
	   term.c_cc[VSUSP]    = 0;     /* Ctrl-z */
	   term.c_cc[VEOL]     = 0;     /* '?0' */
	   term.c_cc[VREPRINT] = 0;     /* Ctrl-r */
	   term.c_cc[VDISCARD] = 0;     /* Ctrl-u */
	   term.c_cc[VWERASE]  = 0;     /* Ctrl-w */
	   term.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
	   term.c_cc[VEOL2]    = 0;     /* '?0' */
#ifdef __QNX__
	term.c_cflag &= ~(IHFLOW | OHFLOW);
#endif

	// settings for qnx
	term.c_cc[VMIN] = 100;
	term.c_cc[VTIME] = 0;

	res = tcsetattr(fdc, TCSANOW, &term);
	if (res < 0) {
		char *pmesg = strerror(errno);
		fprintf(stderr, "failed to tcsetattr(): %s\n", pmesg);
		goto over;
	}

	over: return (res);
}

int ReadCom(int fdc, unsigned short *data) {
	int tick;
	char str[255];
	int n, len;

#define DATA_LENGTH 27
	if (fdc < 0)
		return DATA_LENGTH; // dummy data

	// Data Request
	n = write(fdc, "R", 1);
	//tcdrain(fdc);  // Old legacy that is no longer needed. Besideds that, on QNX6.5.0 this is found taking unnecessarily too long (say 100msec).
	printf("write data ret=%d, fd=%d\n", n, fdc);

	// Get Singale data
	len = 0;
	bzero(str, 27);
	while (len < DATA_LENGTH) {
		n = read(fdc, str + len, DATA_LENGTH - len);
		printf("read data %d (%d)\n", n, len);
		if (n > 0) {
			len += n;
		} else {
			char *pmesg = strerror(errno);
			fprintf(stderr, "failed to read data (ret=%d, fd=%d): %s (%d)\n", n,
					fdc, pmesg, errno);
			goto loop_exit;
		}
	}
	loop_exit: {
		int i;
		for (i = 0; i < DATA_LENGTH; i++) {
			fprintf(stderr, "%02x:", 0x0000ff & str[i]);
		}
		fprintf(stderr, "\n");
	}

	sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx", &tick, &data[0], &data[1],
			&data[2], &data[3], &data[4], &data[5]);

	sprintf(str, "%d,%05d,%05d,%05d,%05d,%05d,%05d\n", tick, data[0], data[1],
			data[2], data[3], data[4], data[5]);

	return len;
}

/*
 * message stuff
 */

#define min(a, b) ((a) < (b) ? (a) : (b))

int io_read(resmgr_context_t * ctp, io_read_t * msg, RESMGR_OCB_T * ocb);	//
static char *buffer = (char *) "Hello world\n";	//

void wait_t(void);		// wait for 3 sec.

static resmgr_connect_funcs_t ConnectFuncs;	//
static resmgr_io_funcs_t IoFuncs;	//
static iofunc_attr_t IoFuncAttr;	//

typedef struct {
	uint16_t msg_no;
	char msg_data[255];
} server_msg_t;

int message_callback(message_context_t * ctp, int type, unsigned flags,
		void *handle) {
	server_msg_t *msg;
	int num;
	char msg_reply[255];

	/* cast a pointer to the message data */
	msg = (server_msg_t *) ctp->msg;

	/* Print out some usefull information on the message */
	//printf( "\n\nServer Got Message:\n" );
	//printf( "  type: %d\n" , type );
	//printf( "  data: %s\n\n", msg->msg_data );
#if 0
	force_sensor_data *data_0;
	force_sensor_data *data_1;
	data_0 = (force_sensor_data *) (Jr3BaseAddress0H + (Jr3DmAddrMask << 2));
	data_1 = (force_sensor_data *) (Jr3BaseAddress1H + (Jr3DmAddrMask << 2));
#endif
	/* Build the reply message */
	num = type - _IO_MAX;
	switch (num) {
	case 1:			// get data
	{
#if 0  // FIX ME
		float tmp[12] = {
			-1.0 * (float) data_0->filter0.fx / (float) data_0->full_scale.fx,
			-1.0 * (float) data_0->filter0.fy / (float) data_0->full_scale.fy,
			-1.0 * (float) data_0->filter0.fz / (float) data_0->full_scale.fz,
			-1.0 * (float) data_0->filter0.mx / (float) data_0->full_scale.mx * 0.1, // Newton*meter*10
			-1.0 * (float) data_0->filter0.my / (float) data_0->full_scale.my * 0.1,
			-1.0 * (float) data_0->filter0.mz / (float) data_0->full_scale.mz * 0.1,
			-1.0 * (float) data_1->filter0.fx / (float) data_1->full_scale.fx,
			-1.0 * (float) data_1->filter0.fy / (float) data_1->full_scale.fy,
			-1.0 * (float) data_1->filter0.fz / (float) data_1->full_scale.fz,
			-1.0 * (float) data_1->filter0.mx / (float) data_1->full_scale.mx * 0.1,
			-1.0 * (float) data_1->filter0.my / (float) data_1->full_scale.my * 0.1,
			-1.0 * (float) data_1->filter0.mz / (float) data_1->full_scale.mz * 0.1
		};
#endif	 
		float tmp[12] = { (force_sensor_data_0[0] - 8192) / 1000.0,
				(force_sensor_data_0[1] - 8192) / 1000.0,
				(force_sensor_data_0[2] - 8192) / 1000.0,
				(force_sensor_data_0[3] - 8192) / 1000.0,
				(force_sensor_data_0[4] - 8192) / 1000.0,
				(force_sensor_data_0[5] - 8192) / 1000.0,
				(force_sensor_data_1[0] - 8192) / 1000.0,
				(force_sensor_data_1[1] - 8192) / 1000.0,
				(force_sensor_data_1[2] - 8192) / 1000.0,
				(force_sensor_data_1[3] - 8192) / 1000.0,
				(force_sensor_data_1[4] - 8192) / 1000.0,
				(force_sensor_data_1[5] - 8192) / 1000.0, };
		memcpy(msg_reply, tmp, sizeof(float) * 12);

	}
		break;
	case 2:			// update offset
#if 0 // FIX ME
	data_0->offsets.fx += data_0->filter0.fx;
	data_0->offsets.fy += data_0->filter0.fy;
	data_0->offsets.fz += data_0->filter0.fz;
	data_0->offsets.mx += data_0->filter0.mx;
	data_0->offsets.my += data_0->filter0.my;
	data_0->offsets.mz += data_0->filter0.mz;
	data_1->offsets.fx += data_1->filter0.fx;
	data_1->offsets.fy += data_1->filter0.fy;
	data_1->offsets.fz += data_1->filter0.fz;
	data_1->offsets.mx += data_1->filter0.mx;
	data_1->offsets.my += data_1->filter0.my;
	data_1->offsets.mz += data_1->filter0.mz;
#endif       
		break;
	case 3:			// set filter
		break;
	case 4:			// get data
#if 0        // FIX ME
	get_force_sensor_info (data_0, msg_reply);
	get_force_sensor_info (data_1, msg_reply + strlen (msg_reply));
#endif       
		break;
	}

	/* Send a reply to the waiting (blocked) client */
	MsgReply(ctp->rcvid, EOK, msg_reply, 255);
	return 0;
}

int main(int argc, char **argv) {
	resmgr_attr_t resmgr_attr;
	message_attr_t message_attr;
	dispatch_t *dpp;
	dispatch_context_t *ctp, *ctp_ret;
	int resmgr_id, message_id;
	int fd1, fd2;

	/* Open Serial port */
	fd1 = open("/dev/serusb1", O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd1 < 0) {
		fprintf(stderr, "could not open /dev/serusb1\n");
	}
	fd2 = open("/dev/serusb2", O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd2 < 0) {
		fprintf(stderr, "could not open /dev/serusb2\n");
	}
        #ifdef __QNX__ 
	    slogf(0, _SLOG_INFO, "Started  fd1 = %d, fd2 = %d\n", fd1, fd2);
        #endif 
	SetComAttr(fd1);
	SetComAttr(fd2);

	/* Create the Dispatch Interface */
	dpp = dispatch_create();
	if (dpp == NULL) {
		fprintf(stderr, "dispatch_create() failed: %s\n", strerror(errno));
		return EXIT_FAILURE;
	}

	memset(&resmgr_attr, 0, sizeof(resmgr_attr));
	resmgr_attr.nparts_max = 1;
	resmgr_attr.msg_max_size = 2048;

	/* Setup the default I/O functions to handle open/read/write/... */
	iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &ConnectFuncs, _RESMGR_IO_NFUNCS,
			&IoFuncs);

	IoFuncs.read = io_read;

	/* Setup the attribute for the entry in the filesystem */
	iofunc_attr_init(&IoFuncAttr, S_IFNAM | 0666, 0, 0);
	IoFuncAttr.nbytes = strlen(buffer) + 1;	//

	resmgr_id = resmgr_attach(dpp, &resmgr_attr, "/dev/jr3q", _FTYPE_ANY, 0,
			&ConnectFuncs, &IoFuncs, &IoFuncAttr);
	if (resmgr_id == -1) {
		fprintf(stderr, "resmgr_attach() failed: %s\n", strerror(errno));
		return EXIT_FAILURE;
	}

	/* Setup Serial Port */

	/* Setup our message callback */
	memset(&message_attr, 0, sizeof(message_attr));
	message_attr.nparts_max = 1;
	message_attr.msg_max_size = 4096;

	/* Attach a callback (handler) for two message types */
	message_id = message_attach(dpp, &message_attr, _IO_MAX + 1, _IO_MAX + 10,
			message_callback, NULL);
	if (message_id == -1) {
		fprintf(stderr, "message_attach() failed: %s\n", strerror(errno));
		return EXIT_FAILURE;
	}

	/* Setup a context for the dispatch layer to use */
	ctp = dispatch_context_alloc(dpp);
	if (ctp == NULL) {
		fprintf(stderr, "dispatch_context_alloc() failed: %s\n",
				strerror(errno));
		return EXIT_FAILURE;
	}

	/* The "Data Pump" - get and process messages */
	while (1) {
		/* do serial read */
		ReadCom(fd1, force_sensor_data_0);
		ReadCom(fd2, force_sensor_data_1);

		printf("dispatch\n");
		/* process message */
		ctp_ret = dispatch_block(ctp);
		if (ctp_ret) {
			dispatch_handler(ctp);
		} else {
			fprintf(stderr, "dispatch_block() failed: %s\n", strerror(errno));
			return EXIT_FAILURE;
		}
	}

	return EXIT_SUCCESS;
}

int io_read(resmgr_context_t * ctp, io_read_t * msg, RESMGR_OCB_T * ocb) {
	int nleft;
	int nbytes;
	int nparts;
	int status;

	if ((status = iofunc_read_verify(ctp, msg, ocb, NULL)) != EOK)
		return (status);

	if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE)
		return (ENOSYS);

	/*
	 * on all reads (first and subsequent) calculate
	 * how many bytes we can return to the client,
	 * based upon the number of bytes available (nleft)
	 * and the client's buffer size
	 */

	nleft = ocb->attr->nbytes - ocb->offset;
	nbytes = min(msg->i.nbytes, nleft);

	if (nbytes > 0) {
		/* set up the return data IOV */
		SETIOV(ctp->iov, buffer + ocb->offset, nbytes);

		/* set up the number of bytes (returned by client's read()) */
		_IO_SET_READ_NBYTES(ctp, nbytes);

		/*
		 * advance the offset by the number of bytes
		 * returned to the client.
		 */

		ocb->offset += nbytes;
		nparts = 1;
	} else {
		/* they've adked for zero bytes or they've already previously
		 * read everything
		 */

		_IO_SET_READ_NBYTES(ctp, 0);
		nparts = 0;
	}

	/* mark the access time as invalid (we just accessed it) */

	if (msg->i.nbytes > 0)
		ocb->attr->flags |= IOFUNC_ATTR_ATIME;

	return (_RESMGR_NPARTS(nparts));

}

void wait_t(void) {
	struct timeval tv, tv1;

	gettimeofday(&tv1, NULL);

	do {
		gettimeofday(&tv, NULL);
	} while (tv.tv_sec - tv1.tv_sec < 3);	// wait for 3 sec.

	return;
}
