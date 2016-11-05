/****************************************************************************
** arduino.c ***********************************************************
****************************************************************************
*
* receive keycodes input via /dev/
*
* Copyright (C) 2016 Jiri Spacek <spacekj3@gmail.com>
*
* Distribute under GPL version 2 or later.
*
*/

#define _GNU_SOURCE 1

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdint.h>
#include <dirent.h>
#include <errno.h>
#include <fnmatch.h>
#include <libgen.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/fcntl.h>
#include <sys/types.h>
#include <termios.h> // POSIX terminal control definitions
#include <linux/input.h>
#include <linux/uinput.h>
#include <sys/stat.h>


#include "lirc_driver.h"

static const logchannel_t logchannel = LOG_DRIVER;

static int arduino_init(void);
//static int devinput_init_fwd(void);
static int arduino_deinit(void);
static int arduino_decode(struct ir_remote* remote, struct decode_ctx_t* ctx);
static char* arduino_rec(struct ir_remote* remotes);
static int drvctl(unsigned int fd, void* arg);
static speed_t get_baudrate(int baud);

enum locate_type {
	locate_by_name,
	locate_by_phys,
	locate_default
};

const struct driver hw_arduino = {
	.name		= "arduino",
	.device		= "/dev/ttyUSB0",
	.features	= LIRC_CAN_REC_LIRCCODE,
	.send_mode	= 0,
	.rec_mode	= LIRC_MODE_LIRCCODE,
	.code_length	= sizeof(uint16_t) * 8,
	.init_func	= arduino_init,
	.deinit_func	= arduino_deinit,
	.open_func	= default_open,
	.close_func	= default_close,
	.send_func	= NULL,
	.rec_func	= arduino_rec,
	.decode_func	= arduino_decode,
	.drvctl_func	= NULL,
	.readdata	= NULL,
	.api_version	= 4,
	.driver_version = "0.9.3",
	.info		= "See file://" PLUGINDOCS "/arduino.html"
};


const struct driver* hardwares[] = { &hw_arduino, (const struct driver*)NULL };


static ir_code code;
static int exclusive = 0;
static int uinputfd = -1;
static struct timeval start, end, last;

static int locate_default_device(char* errmsg, size_t size)
{

	static char devname[256];

	static const char* const DEV_PATTERN =
		"/sys/class/rc/rc0/input[0-9]*/event[0-9]*";
	glob_t matches;
	int r;
	char* event;

	r = glob("/sys/class/rc/rc0/input[0-9]*/event[0-9]*",
		 0, NULL, &matches);
	if (r != 0) {
		log_perror_warn("Cannot run glob %s", DEV_PATTERN);
		snprintf(errmsg, size, "Cannot glob %s", DEV_PATTERN);
		return 0;
	}
	if (matches.gl_pathc == 0) {
		strncpy(errmsg, "No /sys/class/rc/ devices found", size - 1);
		return 0;
	}
	if (matches.gl_pathc > 1) {
		strncpy(errmsg,
			"Multiple /sys/class/rc/ devices found",
			size - 1);
		return 0;
	}
	event = basename(strdupa(matches.gl_pathv[0]));
	snprintf(devname, sizeof(devname), "/dev/input/%s", event);
	drv.device = devname;
	return 1;
}

static int locate_dev(const char* pattern, enum locate_type type)
{
	static char devname[FILENAME_MAX];
	char ioname[255];
	DIR* dir;
	struct dirent* obj;
	int request;

	dir = opendir("/dev/input");
	if (!dir)
		return 1;

	devname[0] = 0;
	switch (type) {
	case locate_by_name:
		request = EVIOCGNAME(sizeof(ioname));
		break;
#ifdef EVIOCGPHYS
	case locate_by_phys:
		request = EVIOCGPHYS(sizeof(ioname));
		break;
#endif
	default:
		closedir(dir);
		return 1;
	}

	while ((obj = readdir(dir))) {
		int fd;

		if (obj->d_name[0] == '.' && (obj->d_name[1] == 0 ||
		    (obj->d_name[1] == '.' && obj->d_name[2] == 0)))
			continue;       /* skip "." and ".." */
		sprintf(devname, "/dev/input/%s", obj->d_name);
		fd = open(devname, O_RDONLY);
		if (!fd)
			continue;
		if (ioctl(fd, request, ioname) >= 0) {
			int ret;

			close(fd);
			ioname[sizeof(ioname) - 1] = 0;
			//ret = !do_match (ioname, pattern);
			ret = fnmatch(pattern, ioname, 0);
			if (ret == 0) {
				drv.device = devname;
				closedir(dir);
				return 0;
			}
		}
		close(fd);
	}

	closedir(dir);
	return 1;
}


int arduino_init(void)
{
	char errmsg[256];
  struct termios toptions;

	log_info("initializing '%s'", drv.device);

	if (strncmp(drv.device, "name=", 5) == 0) {
		if (locate_dev(drv.device + 5, locate_by_name)) {
			log_error("Unable to find '%s'", drv.device);
			return 0;
		}
	} else if (strncmp(drv.device, "phys=", 5) == 0) {
		if (locate_dev(drv.device + 5, locate_by_phys)) {
			log_error("Unable to find '%s'", drv.device);
			return 0;
		}
	} else if (strcmp(drv.device, "auto") == 0) {
		if (locate_default_device(errmsg, sizeof(errmsg)) == 0) {
			log_error(errmsg);
			return 0;
		}
	}
	log_info("Using device: %s", drv.device);
	drv.fd = open(drv.device, O_RDONLY | O_NONBLOCK);
	if (drv.fd < 0) {
		log_error("unable to open '%s'", drv.device);
		return 0;
	}
#ifdef EVIOCGRAB
  // FIXME
  /*	exclusive = 1;
	if (ioctl(drv.fd, EVIOCGRAB, 1) == -1) {
		exclusive = 0;
		log_warn("can't get exclusive access to events coming from `%s' interface", drv.device);
    }*/
#endif

  if (tcgetattr(drv.fd, &toptions) < 0) {
    log_error("Couldn't get term attributes");
    return 0;
  }

  // FIXME to config
  speed_t brate = get_baudrate(9600);
  cfsetispeed(&toptions, brate);
  cfsetospeed(&toptions, brate);

  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  // no flow control
  toptions.c_cflag &= ~CRTSCTS;

  //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

  toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  toptions.c_oflag &= ~OPOST; // make raw

  // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  //toptions.c_cc[VMIN]  = 0;
  //toptions.c_cc[VTIME] = 0;
  //toptions.c_cc[VTIME] = 20;

  tcsetattr(drv.fd, TCSANOW, &toptions);
  if(tcsetattr(drv.fd, TCSAFLUSH, &toptions) < 0) {
    log_error("Couldn't set term attributes");
    return 0;
  }

	return 1;
}

int arduino_deinit(void)
{
	log_info("closing '%s'", drv.device);
	close(drv.fd);
	drv.fd = -1;
	return 1;
}

int arduino_decode(struct ir_remote* remote, struct decode_ctx_t* ctx)
{
	log_trace("arduino_decode");

  if (!map_code(remote, ctx, 0, 0, hw_arduino.code_length, code, 0, 0))
    return 0;

	map_gap(remote, ctx, &start, &last, 0);
	return 1;
}

char* arduino_rec(struct ir_remote* remotes)
{
  uint16_t data;
	int rd;
	ir_code value;
  uint16_t toggle;

	log_trace("arduino_rec");

	last = end;
	gettimeofday(&start, NULL);

	rd = read(drv.fd, &data, sizeof(uint16_t));
	if (rd != sizeof(uint16_t)) {
		log_error("error reading '%s'", drv.device);
		if (rd <= 0 && errno != EINTR)
			arduino_deinit();
		return 0;
	}

  code = (ir_code)(unsigned) data;

	log_trace("code %.8llx", code);

	gettimeofday(&end, NULL);
	return decode_all(remotes);
}

speed_t get_baudrate(int baud)
{
  speed_t brate = baud; // let you override switch below if needed
  switch(baud) {
  case 4800:   brate=B4800;   break;
  case 9600:   brate=B9600;   break;
#ifdef B14400
  case 14400:  brate=B14400;  break;
#endif
  case 19200:  brate=B19200;  break;
#ifdef B28800
  case 28800:  brate=B28800;  break;
#endif
  case 38400:  brate=B38400;  break;
  case 57600:  brate=B57600;  break;
  case 115200: brate=B115200; break;
  }
  return brate;
}
