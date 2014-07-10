/*
 * ResMgr and Message Server Process
 *
 * This program is for JR3/Nitta Force moment sensor.
 * Copyright(C) by Waseda University, Nitta Coropration. 2002.
 *
 * Modified by Kei Okada, 2014
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/mman.h>
#include <hw/pci.h>

#define min(a, b) ((a) < (b) ? (a) : (b))

int io_read (resmgr_context_t * ctp, io_read_t * msg, RESMGR_OCB_T * ocb);	//
static char *buffer = (char *) "Hello world\n";	//

void wait_t (void);		// wait for 3 sec.

#include "QNX101/jr3pci3.idm"

#define VENDORID	0x1762	/* Vendor ID of JR3 */
// #define DEVICEID     0x3114  /* for 4ch board */
// #define DEVICEID     0x3113  /* for 3ch board */
#define DEVICEID	0x3112	/* for 2ch board */
// #define DEVICEID     0x1111  /* for 1ch board */

#define Jr3ResetAddr	0x18000
#define Jr3NoAddrMask	0x40000
#define Jr3DmAddrMask	0x6000
unsigned long MappedAddress;

/* Convert a DSP bus address to a PCI bus address */
#define ToJr3PciAddrH(addr)	(((int)(addr) << 2) + Jr3BaseAddressH)
#define ToJr3PciAddrL(addr)	(((int)(addr) << 2) + Jr3BaseAddressL)

/* Read and Write from the program memory */
#define ReadJr3Pm(addr) 	(*(uint16_t volatile *)(ToJr3PciAddrH((addr))) << 8 |\
				 *(uint8_t  volatile *)(ToJr3PciAddrL((addr))))

#define WriteJr3Pm2(addr,data,data2)	(*(int volatile *)ToJr3PciAddrH((addr)) = (int)(data));(*(int volatile *)ToJr3PciAddrL((addr)) = (int)(data2))



#define WriteJr3Pm(addr,data)		WriteJr3Pm2((addr),(data) >> 8, (data))

/* Read and Write from the data memory using bus relative addressing */
#define	ReadJr3Dm(addr)	(*(uint16_t volatile *)(ToJr3PciAddrH((addr))))
#define WriteJr3Dm(addr,data)	*(int volatile *)(ToJr3PciAddrH((addr))) = (int)(data)

/* Read and Write from the data memory using 0 relative addressing */
#define ReadJr3(addr) (ReadJr3Dm((addr) | Jr3DmAddrMask))
#define WriteJr3(addr,data) WriteJr3Dm((addr) | Jr3DmAddrMask,data)


static resmgr_connect_funcs_t ConnectFuncs;	//
static resmgr_io_funcs_t IoFuncs;	//
static iofunc_attr_t IoFuncAttr;	//

/* Jr3 Base address */
volatile uint32_t Jr3BaseAddressH;
volatile uint32_t Jr3BaseAddressL;
volatile uint32_t Jr3BaseAddress0H;
volatile uint32_t Jr3BaseAddress0L;
volatile uint32_t Jr3BaseAddress1H;
volatile uint32_t Jr3BaseAddress1L;

/* Jr3 memory map */
/*
 For the following structure definitions :
 int:          signed 16 bit value
 unsigned int: unsigned 16-bit value
 bit fields are shown with the lsb first
 */

struct raw_channel
{
  uint16_t raw_time;
  uint16_t dummy1;
  int16_t raw_data;
  int16_t dummy2;
  int16_t reserved[2];
  int16_t dummy3[2];
};

struct force_array
{
  int16_t fx;
  int16_t dummy1;
  int16_t fy;
  int16_t dummy2;
  int16_t fz;
  int16_t dummy3;
  int16_t mx;
  int16_t dummy4;
  int16_t my;
  int16_t dummy5;
  int16_t mz;
  int16_t dummy6;
  int16_t v1;
  int16_t dummy7;
  int16_t v2;
  int16_t dummy8;
};



struct six_axis_array
{
  int16_t fx;
  int16_t dummy1;
  int16_t fy;
  int16_t dummy2;
  int16_t fz;
  int16_t dummy3;
  int16_t mx;
  int16_t dummy4;
  int16_t my;
  int16_t dummy5;
  int16_t mz;
  int16_t dummy6;
};

struct vect_bits
{
  uint8_t fx:1;
  uint8_t fy:1;
  uint8_t fz:1;
  uint8_t mx:1;
  uint8_t my:1;
  uint8_t mz:1;
  uint8_t changedV1:1;
  uint8_t changedV2:1;
  int16_t dummy;
};

struct warning_bits
{
  uint8_t fx_near_set:1;
  uint8_t fy_near_set:1;
  uint8_t fz_near_set:1;
  uint8_t mx_near_set:1;
  uint8_t my_near_set:1;
  uint8_t mz_near_set:1;
  uint8_t reserved:8;
  //uint8_t        reserved : 10; // this will use 3 byte
  int16_t dummy;
};

struct error_bits
{
  uint8_t fx_sat:1;
  uint8_t fy_sat:1;
  uint8_t fz_sat:1;
  uint8_t mx_sat:1;
  uint8_t my_sat:1;
  uint8_t mz_sat:1;
  uint8_t researved:2;
  //uint8_t       researved : 4; // this will use 3 byte
  uint8_t memry_error:1;
  uint8_t sensor_charge:1;
  uint8_t system_busy:1;
  uint8_t cal_crc_bad:1;
  uint8_t watch_dog2:1;
  uint8_t watch_dog:1;
  int16_t dummy1;
};

char *force_units_str[] =
  { (char *) "pound, inch*pound, inch*1000",
(char *) "Newton, Newton*meter*10, mm*10", (char *) "kilogram-force*10, kilogram-Force*cm, mm*10",
(char *) "kilopound, kiloinch*pound, inch*1000" };

enum force_units
{
  lbs_in_lbs_mils,
  N_dNm_mm10,
  kgf10_kgFcm_mm10,
  klbs_kin_lbs_mils,
  reserved_units_4,
  reserved_units_5,
  reserved_units_6,
  reserved_units_7
};

struct thresh_struct
{
  int16_t data_address;
  int16_t dummy1;
  int16_t threshold;
  int16_t dummy2;
  int16_t bit_pattern;
  int16_t dummy3;
};

struct le_struct
{
  int16_t latch_bits;
  int16_t dummy1;
  int16_t number_of_ge_thresholds;
  int16_t dummy2;
  int16_t number_of_le_thresholds;
  int16_t dummy3;
  thresh_struct thresholds[4];
  int16_t reserved;
  int16_t dummy4;
};

enum link_types
{
  end_x_form, tx, ty, tz, rx, ry, rz, neg
};

struct links
{
  link_types link_type;
  int16_t dummy1;
  int16_t link_amount;
  int16_t dummy2;
};

struct transform
{
  links link[8];
};


struct force_sensor_data
{
  raw_channel raw_channels[16];	/* offset 0x0000 */
  char copyright[0x18 * 4];	/* offset 0x0040 */
  int16_t reserved1[0x08 * 2];	/* offset 0x0058 */
  six_axis_array shunts;	/* offset 0x0060 */
  int16_t reserved2[2];		/* offset 0x0066 */
  int16_t dummy1[2];
  six_axis_array default_FS;	/* offset 0x0068 */
  int16_t reserved3;		/* offset 0x006e */
  int16_t dummy2;
  int16_t load_envelope_num;	/* offset 0x006f */
  int16_t dummy3;
  six_axis_array min_full_scale;	/* offset 0x0070 */
  int16_t reserved4;		/* offset 0x0076 */
  int16_t dummy4;
  int16_t transform_num;	/* offset 0x0077 */
  int16_t dummy5;
  six_axis_array max_full_scale;	/* offset 0x0078 */
  int16_t reserved5;		/* offset 0x007e */
  int16_t dummy6;
  int16_t peak_address;		/* offset 0x007f */
  int16_t dummy7;
  force_array full_scale;	/* offset 0x0080 */
  six_axis_array offsets;	/* offset 0x0088 */
  int16_t offset_num;		/* offset 0x008e */
  int16_t dummy8;
  vect_bits vect_axes;		/* offset 0x008f */
  force_array filter0;		/* offset 0x0090 */
  force_array filter1;		/* offset 0x0098 */
  force_array filter2;		/* offset 0x00a0 */
  force_array filter3;		/* offset 0x00a8 */
  force_array filter4;		/* offset 0x00b0 */
  force_array filter5;		/* offset 0x00b8 */
  force_array filter6;		/* offset 0x00c0 */
  force_array rate_data;	/* offset 0x00c8 */
  force_array minimum_data;	/* offset 0x00d0 */
  force_array maximum_data;	/* offset 0x00d8 */
  int16_t near_sat_value;	/* offset 0x00e0 */
  int16_t dummy9;
  int16_t sat_value;		/* offset 0x00e1 */
  int16_t dummy10;
  int16_t rate_address;		/* offset 0x00e2 */
  int16_t dummy11;
  uint16_t rate_divisor;	/* offset 0x00e3 */
  uint16_t dummy12;
  uint16_t rate_count;		/* offset 0x00e4 */
  uint16_t dummy13;
  int16_t command_word2;	/* offset 0x00e5 */
  int16_t dummy14;
  int16_t command_word1;	/* offset 0x00e6 */
  int16_t dummy15;
  int16_t command_word0;	/* offset 0x00e7 */
  uint16_t dummy16;
  uint16_t count1;		/* offset 0x00e8 */
  uint16_t dummy17;
  uint16_t count2;		/* offset 0x00e9 */
  uint16_t dummy18;
  uint16_t count3;		/* offset 0x00ea */
  uint16_t dummy19;
  uint16_t count4;		/* offset 0x00eb */
  uint16_t dummy20;
  uint16_t count5;		/* offset 0x00ec */
  uint16_t dummy21;
  uint16_t count6;		/* offset 0x00ed */
  uint16_t dummy22;
  uint16_t error_count;		/* offset 0x00ee */
  uint16_t dummy23;
  uint16_t count_x;		/* offset 0x00ef */
  uint16_t dummy24;
  warning_bits warnings;	/* offset 0x00f0 */
  error_bits errors;		/* offset 0x00f1 */
  int16_t threshold_bits;	/* offset 0x00f2 */
  int16_t dummy25;
  int16_t last_crc;		/* offset 0x00f3 */
  int16_t dummy26;
  int16_t eeprom_ver_no;	/* offset 0x00f4 */
  int16_t dummy27;
  int16_t software_ver_no;	/* offset 0x00f5 */
  int16_t dummy28;
  int16_t software_day;		/* offset 0x00f6 */
  int16_t dummy29;
  int16_t software_year;	/* offset 0x00f7 */
  int16_t dummy30;
  uint16_t serial_no;		/* offset 0x00f8 */
  uint16_t dummy31;
  uint16_t model_no;		/* offset 0x00f9 */
  uint16_t dummy32;
  int16_t cal_day;		/* offset 0x00fa */
  int16_t dummy33;
  int16_t cal_year;		/* offset 0x00fb */
  int16_t dummy34;
  force_units units;		/* offset 0x00fc */
  int16_t bit;			/* offset 0x00fd */
  int16_t dummy35;
  int16_t channels;		/* offset 0x00fe */
  int16_t dummy36;
  int16_t thickness;		/* offset 0x00ff */
  int16_t dummy37;
  le_struct load_envelopes[0x10];	/* offset 0x0100 */
  transform transforms[0x10];	/* offset 0x0200 */
};


typedef struct
{
  uint16_t msg_no;
  char msg_data[255];
} server_msg_t;


int
download (unsigned int base0, unsigned int base1)
{
  printf ("Download %x %x\n", base0, base1);

  int count;
  int index = 0;
  unsigned int Jr3BaseAddressH;
  unsigned int Jr3BaseAddressL;

  Jr3BaseAddressH = base0;
  Jr3BaseAddressL = base1;

  /* The first line is a line count */
  count = dsp[index++];

  /* Read in file while the count is no 0xffff */
  while (count != 0xffff)
    {
      int addr;

      /* After the count is the address */
      addr = dsp[index++];

      /* loop count times and write the data to the dsp memory */
      while (count > 0)
	{
	  /* Check to see if this is data memory or program memory */
	  if (addr & 0x4000)
	    {
	      int data = 0;

	      /* Data memory is 16 bits and is on one line */
	      data = dsp[index++];
	      WriteJr3Dm (addr, data);
	      count--;
	      if (data != ReadJr3Dm (addr))
		{
		  printf ("data addr: %4.4x out: %4.4x in: %4.4x\n", addr,
			  data, ReadJr3Dm (addr));
		}
	    }
	  else
	    {
	      int data, data2;
	      int data3;


	      /* Program memory is 24 bits and is on two lines */
	      data = dsp[index++];
	      data2 = dsp[index++];
	      WriteJr3Pm2 (addr, data, data2);
	      count -= 2;

	      /* Verify the write */
	      if (((data << 8) | (data2 & 0xff)) !=
		  (data3 = ReadJr3Pm (addr)))
		{
		  //      printf("pro addr: %4.4x out: %6.6x in: %6.6x\n", addr, ((data << 8)|(data2 & 0xff)), /* ReadJr3Pm(addr) */ data3);
		}
	    }
	  addr++;
	}
      count = dsp[index++];
    }


  return 0;

}

void
get_force_sensor_info (force_sensor_data * data, char *msg)
{
  struct tm t1, *soft_day, *cal_day;;
  time_t t2;
  t1.tm_sec = t1.tm_min = t1.tm_hour = 0;
  t1.tm_isdst = -1;
  t1.tm_mon = 0;
  t1.tm_year = data->software_year - 1900;
  t1.tm_mday = data->software_day;
  t2 = mktime (&t1);
  soft_day = localtime (&t2);
  t1.tm_sec = t1.tm_min = t1.tm_hour = 0;
  t1.tm_isdst = -1;
  t1.tm_mon = 0;
  t1.tm_year = data->cal_year - 1900;
  t1.tm_mday = data->cal_day;
  t2 = mktime (&t1);
  cal_day = localtime (&t2);
  snprintf (msg, 128,
	    "rom # %d, soft # %d, %d/%d/%d\n"
	    "serial # %d, model # %d, cal day %d/%d/%d\n"
	    "%s, %d bits, %d ch\n"
	    "(C) %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n\n",
	    data->eeprom_ver_no, data->software_ver_no, soft_day->tm_mday,
	    soft_day->tm_mon + 1, soft_day->tm_year + 1900, data->serial_no,
	    data->model_no, cal_day->tm_mday, cal_day->tm_mon + 1,
	    cal_day->tm_year + 1900, force_units_str[1], data->bit,
	    data->channels, data->copyright[0 * 4 + 1],
	    data->copyright[1 * 4 + 1], data->copyright[2 * 4 + 1],
	    data->copyright[3 * 4 + 1], data->copyright[4 * 4 + 1],
	    data->copyright[5 * 4 + 1], data->copyright[6 * 4 + 1],
	    data->copyright[7 * 4 + 1], data->copyright[8 * 4 + 1],
	    data->copyright[9 * 4 + 1], data->copyright[10 * 4 + 1],
	    data->copyright[11 * 4 + 1], data->copyright[12 * 4 + 1],
	    data->copyright[13 * 4 + 1], data->copyright[14 * 4 + 1],
	    data->copyright[15 * 4 + 1], data->copyright[16 * 4 + 1],
	    data->copyright[17 * 4 + 1], data->copyright[18 * 4 + 1],
	    data->copyright[19 * 4 + 1], data->copyright[20 * 4 + 1],
	    data->copyright[21 * 4 + 1], data->copyright[22 * 4 + 1],
	    data->copyright[23 * 4 + 1]);
}



int
message_callback (message_context_t * ctp, int type, unsigned flags,
		  void *handle)
{
  server_msg_t *msg;
  int num;
  char msg_reply[255];

  /* cast a pointer to the message data */
  msg = (server_msg_t *) ctp->msg;

  /* Print out some usefull information on the message */
  //printf( "\n\nServer Got Message:\n" );
  //printf( "  type: %d\n" , type );
  //printf( "  data: %s\n\n", msg->msg_data );

  force_sensor_data *data_0;
  force_sensor_data *data_1;
  data_0 = (force_sensor_data *) (Jr3BaseAddress0H + (Jr3DmAddrMask << 2));
  data_1 = (force_sensor_data *) (Jr3BaseAddress1H + (Jr3DmAddrMask << 2));

  /* Build the reply message */
  num = type - _IO_MAX;
  switch (num)
    {
    case 1:			// get data
      {
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
	memcpy (msg_reply, tmp, sizeof (float) * 12);
      }
      break;
    case 2:			// update offset
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
      break;
    case 3:			// set filter
      break;
    case 4:			// get data
      get_force_sensor_info (data_0, msg_reply);
      get_force_sensor_info (data_1, msg_reply + strlen (msg_reply));
      break;
    }

  /* Send a reply to the waiting (blocked) client */
  MsgReply (ctp->rcvid, EOK, msg_reply, 256);
  return 0;
}



int
main (int argc, char **argv)
{
  resmgr_attr_t resmgr_attr;
  message_attr_t message_attr;
  dispatch_t *dpp;
  dispatch_context_t *ctp, *ctp_ret;
  int resmgr_id, message_id;

  /* Create the Dispatch Interface */
  dpp = dispatch_create ();
  if (dpp == NULL)
    {
      fprintf (stderr, "dispatch_create() failed: %s\n", strerror (errno));
      return EXIT_FAILURE;
    }

  memset (&resmgr_attr, 0, sizeof (resmgr_attr));
  resmgr_attr.nparts_max = 1;
  resmgr_attr.msg_max_size = 2048;

  /* Setup the default I/O functions to handle open/read/write/... */
  iofunc_func_init (_RESMGR_CONNECT_NFUNCS, &ConnectFuncs,
		    _RESMGR_IO_NFUNCS, &IoFuncs);

  IoFuncs.read = io_read;

  /* Setup the attribute for the entry in the filesystem */
  iofunc_attr_init (&IoFuncAttr, S_IFNAM | 0666, 0, 0);
  IoFuncAttr.nbytes = strlen (buffer) + 1;	//

  resmgr_id = resmgr_attach (dpp, &resmgr_attr, "/dev/jr3q", _FTYPE_ANY,
			     0, &ConnectFuncs, &IoFuncs, &IoFuncAttr);
  if (resmgr_id == -1)
    {
      fprintf (stderr, "resmgr_attach() failed: %s\n", strerror (errno));
      return EXIT_FAILURE;
    }

  /* Setup PCI */
  unsigned int flags = 0;
  unsigned bus, function, index = 0;
  unsigned devid, venid;
  int result;
  unsigned lastbus, version, hardware;
  unsigned int address0;
  void *bufptr;
  unsigned int addr;

  pci_attach (flags);
  result = pci_present (&lastbus, &version, &hardware);
  if (result == -1)
    {
      printf ("PCI BIOS not present!\n");
    }
  devid = DEVICEID;
  venid = VENDORID;
  result = pci_find_device (devid, venid, index, &bus, &function);
  printf ("result PCI %d\n", result);
  printf ("bus %d\n", bus);
  printf ("function %d\n", function);
  pci_read_config_bus (bus, function, 0x10, 1, sizeof (unsigned int), bufptr);

  address0 = *(unsigned int *) bufptr;
  buffer = (char *) bufptr;
  printf ("Board addr 0x%x\n", *(unsigned int *) buffer);

  Jr3BaseAddress0H =
    (volatile uint32_t) mmap_device_memory (NULL, 0x100000,
					    PROT_READ | PROT_WRITE |
					    PROT_NOCACHE, 0, address0);
  Jr3BaseAddress0L =
    (volatile uint32_t) mmap_device_memory (NULL, 0x100000,
					    PROT_READ | PROT_WRITE |
					    PROT_NOCACHE, 0,
					    address0 + Jr3NoAddrMask);
  Jr3BaseAddressH =
    (volatile uint32_t) mmap_device_memory (NULL, 0x100000,
					    PROT_READ | PROT_WRITE |
					    PROT_NOCACHE, 0, address0);
  WriteJr3Dm (Jr3ResetAddr, 0);	// Reset DSP       
  wait_t ();
  download (Jr3BaseAddress0H, Jr3BaseAddress0L);
  wait_t ();

  Jr3BaseAddress1H =
    (volatile uint32_t) mmap_device_memory (NULL, 0x100000,
					    PROT_READ | PROT_WRITE |
					    PROT_NOCACHE, 0,
					    address0 + 0x80000);
  Jr3BaseAddress1L =
    (volatile uint32_t) mmap_device_memory (NULL, 0x100000,
					    PROT_READ | PROT_WRITE |
					    PROT_NOCACHE, 0,
					    address0 + 0x80000 +
					    Jr3NoAddrMask);
  download (Jr3BaseAddress1H, Jr3BaseAddress1L);
  wait_t ();
/*
			Jr3BaseAddress2H = (volatile uint32_t)mmap_device_memory(NULL, 0x100000, PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, address0 + 0x100000); 
			Jr3BaseAddress2L = (volatile uint32_t)mmap_device_memory(NULL, 0x100000, PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, address0 + 0x100000 + Jr3NoAddrMask); 
			download(Jr3BaseAddress2H, Jr3BaseAddress2L);
			wait_t();

			Jr3BaseAddress3H = (volatile uint32_t)mmap_device_memory(NULL, 0x100000, PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, address0 + 0x180000); 
			Jr3BaseAddress3L = (volatile uint32_t)mmap_device_memory(NULL, 0x100000, PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, address0 + 0x180000 + Jr3NoAddrMask); 
			download(Jr3BaseAddress3H, Jr3BaseAddress3L);
			wait_t();
*/

  //reset
  *(uint16_t *) (Jr3BaseAddress0H + ((0x0200 | Jr3DmAddrMask) << 2)) =
    (uint16_t) 0;
  *(uint16_t *) (Jr3BaseAddress0H + ((0x00e7 | Jr3DmAddrMask) << 2)) =
    (uint16_t) 0x0500;


  /* Setup our message callback */
  memset (&message_attr, 0, sizeof (message_attr));
  message_attr.nparts_max = 1;
  message_attr.msg_max_size = 4096;

  /* Attach a callback (handler) for two message types */
  message_id = message_attach (dpp, &message_attr, _IO_MAX + 1,
			       _IO_MAX + 10, message_callback, NULL);
  if (message_id == -1)
    {
      fprintf (stderr, "message_attach() failed: %s\n", strerror (errno));
      return EXIT_FAILURE;
    }

  /* Setup a context for the dispatch layer to use */
  ctp = dispatch_context_alloc (dpp);
  if (ctp == NULL)
    {
      fprintf (stderr, "dispatch_context_alloc() failed: %s\n",
	       strerror (errno));
      return EXIT_FAILURE;
    }

  /* The "Data Pump" - get and process messages */
  while (1)
    {
      ctp_ret = dispatch_block (ctp);
      if (ctp_ret)
	{
	  dispatch_handler (ctp);
	}
      else
	{
	  fprintf (stderr, "dispatch_block() failed: %s\n", strerror (errno));
	  return EXIT_FAILURE;
	}
    }

  return EXIT_SUCCESS;
}


int
io_read (resmgr_context_t * ctp, io_read_t * msg, RESMGR_OCB_T * ocb)
{
  int nleft;
  int nbytes;
  int nparts;
  int status;

  if ((status = iofunc_read_verify (ctp, msg, ocb, NULL)) != EOK)
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
  nbytes = min (msg->i.nbytes, nleft);

  if (nbytes > 0)
    {
      /* set up the return data IOV */
      SETIOV (ctp->iov, buffer + ocb->offset, nbytes);

      /* set up the number of bytes (returned by client's read()) */
      _IO_SET_READ_NBYTES (ctp, nbytes);

      /*
       * advance the offset by the number of bytes
       * returned to the client.
       */

      ocb->offset += nbytes;
      nparts = 1;
    }
  else
    {
      /* they've adked for zero bytes or they've already previously
       * read everything
       */

      _IO_SET_READ_NBYTES (ctp, 0);
      nparts = 0;
    }

  /* mark the access time as invalid (we just accessed it) */

  if (msg->i.nbytes > 0)
    ocb->attr->flags |= IOFUNC_ATTR_ATIME;

  return (_RESMGR_NPARTS (nparts));


}

void
wait_t (void)
{
  struct timeval tv, tv1;

  gettimeofday (&tv1, NULL);

  do
    {
      gettimeofday (&tv, NULL);
    }
  while (tv.tv_sec - tv1.tv_sec < 3);	// wait for 3 sec.

  return;
}
