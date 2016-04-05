// test-com.c
#define 	DEBUGSS	0

#include	<stdio.h>
#include	<fcntl.h>
#include	<errno.h>
#include	<time.h>
#include	<termios.h>
#include	<string.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/slog.h>
#include <sys/time.h>

#define true		1
#define false		0

int kbhit(void) {

	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF) {

		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

#define cfsetspeed(term, baudrate)\
   cfsetispeed(term, baudrate);\
   cfsetospeed(term, baudrate);

int com_write(int fd, const char* msg) {
	// IO is currently non-blocking. This is what we want for the more common read case.
	int origflags = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, origflags & ~O_NONBLOCK); // @todo can we make this all work in non-blocking?
	ssize_t len = strlen(msg);
	ssize_t retval = write(fd, msg, len);
	int fputserrno = errno;
	fcntl(fd, F_SETFL, origflags | O_NONBLOCK);
	errno = fputserrno; // Don't want to see the fcntl errno below.

	if (retval != -1) {
		return retval;
	}
	printf("fputs fialed %d: %s", errno, strerror(errno));
	return -1;
}

int main() {
	int status;
	//FILE			*fd;
	int fd;
	char fname[64];
	char devname_in[64];
	char devname_out[64];
	char str[256];
	unsigned short data[6];
	int comNo;
	int tick;
	int clk, clkb, clkb2, clk0;
	int tw;
	int num;
	int n, len;

	//fd = NULL;
	fd = -1;

	start:
	// Open COM port
	printf("Enter COM port > ");
	scanf("%d", &comNo);
	comNo = 1;
	sprintf(devname_out, "/dev/serusb%d", comNo);
	printf("Open %s for output\n", devname_out);
	fd = open(devname_out, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0 || fd < 0)
		goto over;

	// Get Sampling Frequency
	tw = 16;
	printf("Enter sampling time (ms) > ");
	fflush(stdout);
	//scanf("%d", &tw);
	printf("Sampling time = %d ms\n", tw);

	printf("Enter File name > ");
	fflush(stdout);
	//scanf("%s", fname);
	//fd = fopen(fname, "w");
	//fd = fopen("test.txt", "w");
	//if (!fd)
	//	goto over;

	slogf(0, _SLOG_INFO, "Started  input: %s fd = %d\n", devname_in, fd);
	slogf(0, _SLOG_INFO, "        output: %s fd = %d\n", devname_out, fd);
	// Set Commport Settings
	SetComAttr(fd);

	// Read Data
	printf("=== record data ===\n");
	clk0 = clock() / (CLOCKS_PER_SEC / 1000);
	clkb = 0;
	clkb2 = 0;
	num = 0;

	// Data Requests
	struct timeval wr0, wr1, tcd0, tcd1;
	gettimeofday(&wr0, NULL);
	n = write(fd, "R", 1);
	gettimeofday(&wr1, NULL);
	gettimeofday(&tcd0, NULL);
	tcdrain(fd);
	gettimeofday(&tcd1, NULL);
#define DELTA_SEC(start, end) (end.tv_sec - start.tv_sec + (end.tv_usec - start.tv_usec)/1e6)
	printf("write took %7.3f, tcdrain took %7.3f [msec]\n", DELTA_SEC(wr0, wr1) * 1000, DELTA_SEC(tcd0, tcd1) * 1000);
	printf("write data (ret %d)\n", n);

	while (true) {
		// Wait for Sampling rate
		while (true) {
			clk = clock() / (CLOCKS_PER_SEC / 1000) - clk0;
			if (clk >= clkb + tw) {
				clkb = clk / tw * tw;
				break;
			}
		}

		// Data Request
		gettimeofday(&wr0, NULL);
		n = write(fd, "R", 1);
		gettimeofday(&wr1, NULL);
		printf("write (in loop) took %7.3f [msec]\n", DELTA_SEC(tcd0, tcd1) * 1000);
		gettimeofday(&tcd0, NULL);
		tcdrain(fd);  //  The tcdrain() function waits until all output has been physically transmitted to the device associated with fd, or until a signal is received.
		gettimeofday(&tcd1, NULL);
		printf("(In loop) write took %7.3f, tcdrain took %7.3f [msec]\n", DELTA_SEC(wr0, wr1) * 1000, DELTA_SEC(tcd0, tcd1) * 1000);
		printf("write data (ret %d)\n", n);

		// Get Single data
#define DATA_LENGTH 27
		len = 0;
		bzero(str, 27);  // Setting the first 27 bytes of the area starting `str` to zero.
		while (len < DATA_LENGTH) {
			n = read(fd, str + len, DATA_LENGTH - len);
			printf("read data (ret %d, %d bytes read))\n", n, len + n);
			if (n > 0) {
				len += n;
			} else {
				char *pmesg = strerror(errno);
				fprintf(stderr, "failed to read data (%d): %s (%d)\n", n, pmesg,
						errno);
				usleep(10000);
				//goto loop_exit;
			}
		}
		//goto skip;
		loop_exit: {
			int i;
			for (i = 0; i < DATA_LENGTH; i++) {
				fprintf(stderr, "%02x:", 0x0000ff & str[i]);
			}
		}

		sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx", &tick, &data[0], &data[1],
				&data[2], &data[3], &data[4], &data[5]);

		sprintf(str, "%05d,%d,%05d,%05d,%05d,%05d,%05d,%05d\n", clk / tw * tw,
				tick, data[0], data[1], data[2], data[3], data[4], data[5]);

		fprintf(stderr, str);
		//fprintf(fd, str);
		num++;

		skip: if (clk >= 10000)
			break;

		// Dsiaply Console
		if (clk >= clkb2 + 1000) {
			printf(str);
			if (kbhit() && getchar() == '.')
				break;
			clkb2 = clk / 1000 * 1000;
		}
	}

	over: if (fd >= 0) {
		close(fd);
		fd = -1;
	}

	printf("=== num = %d ===\n", num);

	printf("exit (y / n) ? > ");
	fflush(stdout);
	exit(0);   //scanf("%s", str);
	if (str[0] == 'y') {
//		exit(0);
	} else {
		goto start;
	}
}

void clear_packet(int fd) {

	// clear existing packet
	int oldf = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, oldf | O_NONBLOCK);
	unsigned char c;
	while (read(fd, &c, 1) != EOF)
		;
	fcntl(fd, F_SETFL, oldf);
}

//#define  B921600  B57600
#define   B921600 0010007

int SetComAttr2(int fd) {
	// Settings for USB?
	struct termios newtio;
	tcgetattr(fd, &newtio);
	memset(&newtio.c_cc, 0, sizeof(newtio.c_cc));
	newtio.c_cflag |= CS8 | CLOCAL | CREAD;
	//newtio.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
	newtio.c_cflag &= ~(IHFLOW | OHFLOW);
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;   //ICANON;
	//newtio.c_lflag |= IEXTEN; // QNX?
	//newtio.c_lflag = 0;

	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] = 2;
	newtio.c_cc[VEOF] = 4; /* Ctrl-d */

	//printf("speed = %04o %d\n", B9600, B9600);
	//printf("speed = %04o %d\n", B115200, B9600);
	//printf("speed = %04o %d\n", B921600, B9600);
#if 0
	int n;
	n = cfsetspeed(&newtio, 921600L);
	//n = cfsetspeed(&newtio, 115200);
	if (n<0)
	{
		char *pmesg = strerror(errno);
		fprintf (stderr, "failed to cfsetspeed(): %s\n", pmesg);
		exit(-1);
	}
#endif   
	// activate new settings
	tcflush(fd, TCIFLUSH);
	if (tcsetattr(fd, TCSANOW, &newtio) < 0) {
		printf("can not set attr");
		exit(-1);
	}

	sleep(1);

	int retval = tcflush(fd, TCIOFLUSH);
	if (retval != 0) {
		printf("tcflush failed\n");
	}
}

int SetComAttr4(int fdc) {

	int n;

	struct termios term;

	n = tcgetattr(fdc, &term);
	if (n < 0) {
		char *pmesg = strerror(errno);
		fprintf(stderr, "failed to tcgetattr(): %s\n", pmesg);
		goto over;
	}

	bzero(&term, sizeof(term));

	//term.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	term.c_cflag = CS8 | CLOCAL | CREAD;
#if 1
	//n = cfsetspeed(&term, 115200L);
	//n = cfsetspeed(&term, 230400L);
	//n = cfsetspeed(&term, 460800L);
	n = cfsetspeed(&term, 921600L)
	;
	//n = cfsetspeed(&term, 115200);
	//n = cfsetspeed(&term, 19200);
	if (n < 0) {

		char *pmesg = strerror(errno);
		fprintf(stderr, "failed to cfsetspeed(): %s\n", pmesg);
		goto over;
	}
#endif
	term.c_iflag = IGNPAR;
	term.c_oflag = 0;
	term.c_lflag = 0;/*ICANON;*/
	//term.c_lflag = 0;/*ICANON;*/
	term.c_lflag |= IEXTEN; // QNX?

	term.c_cc[VINTR] = 0; /* Ctrl-c */
	term.c_cc[VQUIT] = 0; /* Ctrl-? */
	term.c_cc[VERASE] = 0; /* del */
	term.c_cc[VKILL] = 0; /* @ */
	term.c_cc[VEOF] = 4; /* Ctrl-d */
	term.c_cc[VTIME] = 0;
	term.c_cc[VMIN] = 0;
	//term.c_cc[VSWTC]    = 0;     /* '?0' */
	term.c_cc[VSTART] = 0; /* Ctrl-q */
	term.c_cc[VSTOP] = 0; /* Ctrl-s */
	term.c_cc[VSUSP] = 0; /* Ctrl-z */
	term.c_cc[VEOL] = 0; /* '?0' */
	term.c_cc[VREPRINT] = 0; /* Ctrl-r */
	term.c_cc[VDISCARD] = 0; /* Ctrl-u */
	term.c_cc[VWERASE] = 0; /* Ctrl-w */
	term.c_cc[VLNEXT] = 0; /* Ctrl-v */
	term.c_cc[VEOL2] = 0; /* '?0' */

#ifdef __QNX__
	term.c_cflag &= ~PARENB;           // disable parity check
	term.c_cflag &= ~CSTOPB;// 1 stop bit
	term.c_cflag &= ~(IHFLOW | OHFLOW);
	// settings for qnx
	term.c_cc[VMIN] = 1;
	term.c_cc[VTIME] = 0;
#endif

	//tcflush(fdc, TCIFLUSH);
	n = tcsetattr(fdc, TCSANOW, &term);
	if (n < 0) {

		char *pmesg = strerror(errno);
		fprintf(stderr, "failed to tcsetattr(): %s\n", pmesg);
		goto over;
	}

	over:

	sleep(2);
	return (n);
}

int SetComAttr(int fdc) {
	if (fdc < 0) {
		char *pmesg = strerror(errno);
		fprintf(stderr, "failed to open : %s\n", pmesg);
		goto over;
	}

	struct termios term;
	int res = tcgetattr(fdc, &term);
	if (res < 0) {
		char *pmesg = strerror(errno);
		fprintf(stderr, "failed to tcgetattr(): %s\n", pmesg);
		goto over;
	}
	cfmakeraw(&term);
	res = cfsetspeed(&term, 921600)
	;
//	 res = cfsetspeed(&term, 230400);
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
	term.c_cc[VINTR] = 0; /* Ctrl-c */
	term.c_cc[VQUIT] = 0; /* Ctrl-? */
	term.c_cc[VERASE] = 0; /* del */
	term.c_cc[VKILL] = 0; /* @ */
	term.c_cc[VEOF] = 4; /* Ctrl-d */
	term.c_cc[VTIME] = 0;
	term.c_cc[VMIN] = 1;
	//term.c_cc[VSWTC]    = 0;     /* '?0' */
	term.c_cc[VSTART] = 0; /* Ctrl-q */
	term.c_cc[VSTOP] = 0; /* Ctrl-s */
	term.c_cc[VSUSP] = 0; /* Ctrl-z */
	term.c_cc[VEOL] = 0; /* '?0' */
	term.c_cc[VREPRINT] = 0; /* Ctrl-r */
	term.c_cc[VDISCARD] = 0; /* Ctrl-u */
	term.c_cc[VWERASE] = 0; /* Ctrl-w */
	term.c_cc[VLNEXT] = 0; /* Ctrl-v */
	term.c_cc[VEOL2] = 0; /* '?0' */
#ifdef __QNX__
	term.c_cflag &= ~(IHFLOW | OHFLOW);
#endif

	// settings for qnx
	term.c_cc[VMIN] = 0;
	term.c_cc[VTIME] = 2;

	res = tcsetattr(fdc, TCSANOW, &term);
	if (res < 0) {
		char *pmesg = strerror(errno);
		fprintf(stderr, "failed to tcsetattr(): %s\n", pmesg);
		goto over;
	}

	over: return (res);
}

