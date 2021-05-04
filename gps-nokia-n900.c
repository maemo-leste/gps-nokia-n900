/* -*- linux-c -*-
 * GPS handling for N900
 *
 * Copyright Luke Dashjr
 *
 * Source originally from: http://dashjr.org/~luke-jr/tmp/code/gps3.c
 *
 * Distribute under GPLv3.
 *
 * gcc -std=c99 gps3.c -o gps3
 *

This will bind to /dev/pts/0 (or similar). It needs ofone to be
started, and modem to be online.  You can verify correct operation by

gpsd -gd
cat < /dev/pts/X

(You should see debug messages at that point).

gps3 -d starts gpsd automatically.

GPS_PTY_LINK=/tmp/gps_source ./gps3 -dg
cat < /tmp/gps_source

 */

#include <assert.h>
#include <errno.h>
#include <math.h>
#include <pty.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <fcntl.h>
#include <linux/phonet.h>

#define BUFFER_SIZE 4096

static int sck;
static char isActive = 0, nopm = 0, nogpsd = 0, do_fake = 0, isQuiet;

#ifdef DEBUG
#	define debug(...) printf(__VA_ARGS__)
#else
#	define debug(...)
#endif

static void myHexDump(const char *buf, size_t buflen, uint8_t indent)
{
#ifdef DEBUG
	if (buflen == 0) {
		debug("00000000  (Null Length)\n");
		return;
	}
	char newline[indent + 7];
	newline[0] = '\n';
	memset(&(newline[1]), '\t', indent);
	strcpy(&(newline[indent + 1]), "%08x ");
	for (size_t xdo = 0; xdo < buflen; ++xdo) {
		switch (xdo % 16) {
		case 0:
			debug(xdo ? newline : (newline + 1), xdo);
			break;
		case 8:
			debug(" ");
			break;
		}
		debug(" %02x", buf[xdo]);
	}
	debug("\n");
#endif
}

int cellmoStartGPS(int sck)
{
	struct sockaddr_pn sa;
	memset(&sa, 0, sizeof(sa));
	sa.spn_family = AF_PHONET;

	sa.spn_resource = 16;
	if (4 !=
	    sendto(sck, "\0\20\1T", 4, 0, (struct sockaddr *)&sa, sizeof(sa)))
		return -1;
	sa.spn_resource = 84;
	if (28 !=
	    sendto(sck,
		   "\0\220\0\1\0\0\0\0\0\0\0\0\t\1\0\20\0\0\0\n\0\0\0\3\0\0\0\0",
		   28, 0, (struct sockaddr *)&sa, sizeof(sa)))
		return -2;

	return 0;
}

int cellmoStopGPS(int sck)
{
	struct sockaddr_pn sa;
	memset(&sa, 0, sizeof(sa));
	sa.spn_family = AF_PHONET;
	sa.spn_resource = 84;
	if (28 !=
	    sendto(sck,
		   "\0\220\1\1\0\0\0\0\0\0\0\5\t\1\0\20\0\0\0\1\0\0\0\3\0\n\0\0",
		   28, 0, (struct sockaddr *)&sa, sizeof(sa)))
		return -1;
	if (12 !=
	    sendto(sck, "\0\220\2\0\0\0\0\0\0\0\0\5", 12, 0,
		   (struct sockaddr *)&sa, sizeof(sa)))
		return -2;
	return 0;
}

int setupGpsPty()
{
	int ptyMaster;
	ptyMaster = posix_openpt(O_RDWR);
	if (ptyMaster < 0)
		return -1;
	{
		struct termios tios;
		if (tcgetattr(ptyMaster, &tios))
			return -5;
		cfmakeraw(&tios);
		if (tcsetattr(ptyMaster, TCSANOW, &tios))
			return -6;
	}
	{
		int flags;
		flags = fcntl(ptyMaster, F_GETFL);
		if (flags < 0)
			return -7;
		flags |= O_NONBLOCK;
		if (fcntl(ptyMaster, F_SETFL, flags) == -1)
			return -8;
	}
	if (grantpt(ptyMaster))
		return -2;
	if (unlockpt(ptyMaster))
		return -3;
	const char *ptySlaveLink = getenv("GPS_PTY_LINK");

	{
		const char *ptySlaveName = ptsname(ptyMaster);

		fprintf(stderr, "got slave name %s\n", ptySlaveName);
		if (!nogpsd) {
			char cmd[10240];
			snprintf(cmd, 10239, "/usr/sbin/gpsd -N %s &",
				 ptySlaveName);
			system(cmd);
		}
	}

	if (ptySlaveLink && *ptySlaveLink) {
		const char *ptySlaveName = ptsname(ptyMaster);
		if ((!ptySlaveName)
		    || ((0 != unlink(ptySlaveLink)) && errno != ENOENT)
		    || (0 != symlink(ptySlaveName, ptySlaveLink))) {
			close(ptyMaster);
			return -4;
		}
	}
	return ptyMaster;
}

static void waitForSlave(int ptyMaster)
{
	// HACK, because there's no sane way to detect this :(
	fd_set rfds;
	FD_ZERO(&rfds);
	struct timeval tv;
	while (1) {
		write(ptyMaster, "$GP\r\n", 5);
		debug("Waiting for slave...\n");
		FD_SET(ptyMaster, &rfds);
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		if (select(ptyMaster + 1, &rfds, NULL, NULL, &tv))
			break;
	}
}

// BEGIN: Lifted from gpsd-2.32/nmea_parse.c
void nmea_add_checksum(char *sentence)
/* add NMEA checksum to a possibly  *-terminated sentence */
{
	unsigned char sum = '\0';
	char c, *p = sentence;

	if (*p == '$') {
		p++;
	} else {
#if 0
		gpsd_report(1, "Bad NMEA sentence: '%s'\n", sentence);
#endif
	}
	while (((c = *p) != '*') && (c != '\0')) {
		sum ^= c;
		p++;
	}
	*p++ = '*';
	(void)snprintf(p, 5, "%02X\r\n", (unsigned)sum);
}

int nmea_send(int fd, const char *fmt, ...)
/* ship a command to the GPS, adding * and correct checksum */
{
	int status;
	char buf[BUFSIZ];
	va_list ap;

	va_start(ap, fmt);
	(void)vsnprintf(buf, sizeof(buf) - 5, fmt, ap);
	va_end(ap);
	if (fmt[0] == '$') {
		strcat(buf, "*");
		nmea_add_checksum(buf);
	} else
		strcat(buf, "\r\n");
	status = (int)write(fd, buf, strlen(buf));
	if (status == (int)strlen(buf)) {
#if 0
		gpsd_report(2, "=> GPS: %s\n", buf);
#endif
		return status;
	} else {
#if 0
		gpsd_report(2, "=> GPS: %s FAILED\n", buf);
#endif
		return -1;
	}
}

//  END : Lifted from gpsd-2.32/nmea_parse.c

static void handlePhonetPacket(int sck, int pty)
{
	char buf[BUFFER_SIZE];
	ssize_t buflen = recv(sck, buf, sizeof(buf), 0);
	assert(buflen > 0);
	debug("Read %ld bytes : ", buflen);

	if (buflen < 9)
		return;
	uint8_t spcount = buf[8];
	debug("Total %d subpackets\n", spcount);

	uint8_t validData = 0;
	float lat = 0.0, lon = 0.0, alti = 0.0, epv2 = 0.0, track = 0.0;
	uint32_t eph = 0;
	uint16_t date_year = 0, time_ms = 0, time_accuracy = 0, speed = 0;
	uint8_t date_month = 0, date_day = 0, time_hour = 0;
	uint8_t time_minute = 0, time_second = 0, satVisible = 0;
	int8_t usedSats[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	uint8_t splen;
	for (size_t spoffset = 12; spoffset < buflen; spoffset += splen) {
		char *sp = buf + spoffset;
		assert(sp[0] == 9);
		uint8_t sptype = sp[1];
		assert(sp[2] == 0);
		splen = sp[3];

		char *spd = sp + 4;
		switch (sptype) {
		case 2:	// Position: Latitude, longitude, altitude
			{
				validData |= 1;
				debug("    Subpacket: Position\n");
				assert(splen >= 18);

				lat =
				    360. * (((float)spd[0] / 0x100) +
					    ((float)spd[1] / 0x10000) +
					    ((float)spd[2] / 0x1000000) +
					    ((float)spd[3] / 0x100000000));
				if (lat > 180.)
					lat -= 360.;
				debug("\tLatitude : %f\n", lat);
				lon =
				    360. * (((float)spd[4] / 0x100) +
					    ((float)spd[5] / 0x10000) +
					    ((float)spd[6] / 0x1000000) +
					    ((float)spd[7] / 0x100000000));
				if (lon > 180.)
					lon -= 360.;
				debug("\tLongitude: %f\n", lon);

				// 8,9,10,11 = Latitude error ?

				eph =
				    (spd[12] << 24) | (spd[13] << 16) | (spd[14]
									 << 8) |
				    spd[15];
				debug("\teph: %dcm\n", eph);

				// 16,17 ?
				debug("\tunknown: %lx %lx\n", spd[16], spd[17]);

				uint16_t altiA = (spd[18] << 8) | spd[19];
				uint16_t altiB = (spd[22] << 8) | spd[23];
				alti =
				    (float)(altiA - *((int16_t *) & altiB)) / 2;
				epv2 = ((spd[20] << 8) | spd[21]) / 2;
				debug("\tAltitude: %.1fm (accurate to %.1f?)\n",
				      alti, epv2);

				break;
			}
		case 3:	// Date and time
			{
				validData |= 2;
				debug("    Subpacket: DateTime\n");

				date_year = (spd[0] << 8) | spd[1];
				date_month = spd[2];
				date_day = spd[3];
				debug("\tDate: %04d-%02d-%02d\n", date_year,
				      date_month, date_day);

				// 4 ?

				time_hour = spd[5];
				time_minute = spd[6];
				// 7 ?
				time_ms = (spd[8] << 8) | spd[9];
				time_second = time_ms / 1000;
				time_ms %= 1000;
				time_accuracy = (spd[10] << 8) | spd[11];
				debug
				    ("\tTime: %02d:%02d:%02d.%03d (accuracy: %d)\n",
				     time_hour, time_minute, time_second,
				     time_ms, time_accuracy);

				break;
			}
		case 4:	// Track, speed, climb
			{
				validData |= 4;
				debug("    Subpacket: Motion\n");

				track = ((spd[0] << 8) | spd[1]) / 100.;
				float epd = ((spd[2] << 8) | spd[3]) / 100.;
				debug
				    ("\tDirection of motion: %.2f degrees (accurate to %.2f degrees?)\n",
				     track, epd);

				// 4,5?

				speed = (spd[6] << 8) | spd[7];
				uint16_t eps = (spd[8] << 8) | spd[9];
				debug
				    ("\tSpeed: %d cm/sec (accurate to %d cm/sec)\n",
				     speed, eps);

				uint16_t climbA = (spd[10] << 8) | spd[11];
				int16_t climb = *((int16_t *) & climbA);
				uint16_t epc = (spd[12] << 8) | spd[13];
				debug
				    ("\tClimb: %d cm/sec (accurate to %d cm/sec)\n",
				     climb, epc);

				// 14,15?

				break;
			}
		case 5:	// Sat info???
			{
				validData |= 8;
				debug("    Subpacket: Satellites\n");

				satVisible = spd[0];
				debug("\tSatellites Visible: %d\n", satVisible);

				int8_t *nextUsed = &(usedSats[0]);
				for (size_t sos = 8, i = 1; sos < splen;
				     sos += 12, ++i) {
					char *sd = sp + sos;
					// 0 ?
					uint8_t PRN = sd[1];
					uint8_t inUse = sd[2];
					float signalStrength =
					    (float)((sd[3] << 8) | sd[4]) /
					    100.0;
					// 5 ?
					float elevation =
					    ((sd[6] << 8) | sd[7]) / 100.0;
					float azimuth =
					    ((sd[8] << 8) | sd[9]) / 100.0;
					// 10,11 ?
					debug
					    ("\tSatellite: [%c] PRN=%2d signal=%.02f%% azimuth=%6.2f elevation=%5.2f\n",
					     inUse ? 'x' : '_', PRN,
					     signalStrength, azimuth,
					     elevation);
					if (isQuiet < 2) {
						nmea_send(pty,
							  "$GPGSV,%d,%d,%d,%02d,%02d,%03d,%02d",
							  satVisible, i,
							  satVisible, PRN,
							  (uint8_t) elevation,
							  (uint8_t) azimuth,
							  (uint8_t)
							  signalStrength);
					}
					if (inUse)
						*(nextUsed++) = PRN;
				}
				break;
			}
		case 7:	// CellInfoGSM
			{
				validData |= 16;
				debug("    Subpacket: CellInfoGSM\n");
				uint16_t mcc = (spd[0] << 8) | spd[1];
				debug("\tMobile Country Code: %d\n", mcc);
				uint16_t mnc = (spd[2] << 8) | spd[3];
				debug("\tMobile Network Code: %d\n", mnc);
				uint16_t lac = (spd[4] << 8) | spd[5];
				debug("\tLocation Area  Code: %d\n", lac);
				uint16_t cellId = (spd[6] << 8) | spd[7];
				debug("\tCell ID: %d\n", cellId);
				break;
			}
		case 8:	// CellInfoWCDMA
			{
				validData |= 32;
				debug("    Subpacket: CellInfoWCDMA\n");
				uint16_t mcc = (spd[0] << 8) | spd[1];
				debug("\tMobile Country Code: %d\n", mcc);
				uint16_t mnc = (spd[2] << 8) | spd[3];
				debug("\tMobile Network Code: %d\n", mnc);
				uint32_t ucid =
				    (spd[4] << 24) | (spd[5] << 16) | (spd[6] <<
								       8) |
				    spd[7];
				debug("\tUC ID: %d\n", ucid);
				break;
			}
		default:
			debug
			    ("\tSubpacket: Unknown (type 0x%02x), length %d:\n",
			     sptype, splen);
			myHexDump(sp, splen, 2);
		}
	}

	if (isQuiet > 1)
		return;
	if (validData & 8) {
		char GPGSAsats[37];
		char *p = &(GPGSAsats[0]);
		for (int i = 0; i < 12; ++i)
			if (usedSats[i])
				p += sprintf(p, ",%d", usedSats[i]);
			else
				*(p++) = ',';
		*p = '\0';
		if (isQuiet < 2) {
			nmea_send(pty, "$GPGSA,A,%c%s,,,",
				  (validData & 1) ? '3' : '1', GPGSAsats);
		}
	}
	if ((validData & 3) == 3) {
		uint8_t latD = abs((int16_t) lat);
		float latM = (fabs(lat) - (float)latD) * 60;
		uint8_t lonD = abs((int16_t) lon);
		float lonM = (fabs(lon) - (float)lonD) * 60;

		//nmea_send(pty, "$GPGLL,%d%09.6f,%c,%d%09.6f,%c,%02d%02d%02d.%02d,A", latD, latM, (lat < 0) ? 'S' : 'N', lonD, lonM, (lon < 0) ? 'W' : 'E', time_hour, time_minute, time_second, time_ms / 10);

		if (!isQuiet || (validData & 8))
			nmea_send(pty,
				  "$GPGGA,%02d%02d%02d.%02d,%d%09.6f,%d,%d%09.6f,%c,%d,%d,%.1f,%.1f,1,0,0,,",
				  time_hour, time_minute, time_second,
				  time_ms / 10, latD, latM,
				  (lat < 0) ? 'S' : 'N', lonD, lonM,
				  (lon < 0) ? 'W' : 'E', !!(validData & 8),
				  satVisible, (float)eph / 100.0, alti);

		if ((validData & 12) == 12) {
			float speedKnots = (float)speed / (463. / 9.);
			nmea_send(pty,
				  "$GPRMC,%02d%02d%02d.%02d,A,%d%09.6f,%c,%d%09.6f,%c,%f,%f,%02d%02d%02d,,",
				  time_hour, time_minute, time_second,
				  time_ms / 10, latD, latM,
				  (lat < 0) ? 'S' : 'N', lonD, lonM,
				  (lon < 0) ? 'W' : 'E', speedKnots, track,
				  date_day, date_month, date_year % 100);
		}
	}
}

static void handleFakePacket(int sck, int pty)
{
	char buf[BUFFER_SIZE];

	ssize_t buflen = read(sck, buf, sizeof(buf));
	assert(buflen > 0);
	debug("Read %ld bytes : ", buflen);
	buf[buflen] = 0;

	float lat, lon;

	if (buf[0] == 'l') {
		sscanf(buf + 2, "%f %f\n", &lat, &lon);

		uint8_t latD = abs((int16_t) lat);
		float latM = (fabs(lat) - (float)latD) * 60;
		uint8_t lonD = abs((int16_t) lon);
		float lonM = (fabs(lon) - (float)lonD) * 60;
		int time_hour = 12, time_minute = 34, time_second =
		    56, time_ms = 0;

		debug("Have %f %f\n", lat, lon);

		nmea_send(pty,
			  "$GPGLL,%d%09.6f,%c,%d%09.6f,%c,%02d%02d%02d.%02d,A",
			  latD, latM, (lat < 0) ? 'S' : 'N', lonD, lonM,
			  (lon < 0) ? 'W' : 'E', time_hour, time_minute,
			  time_second, time_ms / 10);
		return;
	}
	if (buf[0] == 'n') {
		nmea_send(pty, "%s", buf + 1);
		return;
	}
	if (buf[0] == 'q') {
		/* XXX: Should this be '=' and not '==' ? */
		isQuiet == buf[1] - '0';
		return;
	}
}

static void sigShutdown(int signum)
{
	debug("Shutdown signal received\n");
	if (isActive)
		cellmoStopGPS(sck);
	exit(0);
}

void parse_opts(int argc, char *argv[])
{
	int opt;

	while ((opt = getopt(argc, argv, "dgf")) != -1) {
		switch (opt) {
		case 'd':
			nopm = 1;
			break;
		case 'g':
			nogpsd = 1;
			break;
		case 'f':
			do_fake = 1;
			break;
		default:	/* '?' */
			fprintf(stderr, "Usage: %s [-dgf]\n", argv[0]);
			exit(EXIT_FAILURE);
		}
	}
}

int main(int argc, char *argv[])
{
	parse_opts(argc, argv);

	signal(SIGHUP, sigShutdown);
	signal(SIGINT, sigShutdown);
	signal(SIGQUIT, sigShutdown);
	signal(SIGPIPE, sigShutdown);

	sck = socket(AF_PHONET, SOCK_DGRAM, 0);
	assert(sck >= 0);
	{
		struct sockaddr_pn sa;
		memset(&sa, 0, sizeof(sa));
		sa.spn_family = AF_PHONET;
		sa.spn_resource = 255;
		assert(!bind(sck, (struct sockaddr *)&sa, sizeof(sa)));
	}

	int pty = setupGpsPty();
	assert(pty >= 0);

	int maxfd = pty;
	if (sck > maxfd)
		maxfd = sck;

	int fake = -1;
	if (do_fake) {
		fake = open("/tmp/gpsfake_pipe", O_RDWR);
		assert(fake >= 0);
		if (fake > maxfd)
			maxfd = fake;
	}

	++maxfd;

	while (1) {
		int ptySlave = open(ptsname(pty), O_RDWR | O_NOCTTY);
		assert(ptySlave >= 0);
		// 1. Select on READ until gpsd probes, writing "$GP\r\n" every so often
		if (!nopm)
			waitForSlave(pty);
		// 2. Close our slave fd
		close(ptySlave);
		// 3. ACTIVATE AND RUN
		debug("TTY active, starting GPS\n");
		assert(!cellmoStartGPS(sck));
		isActive = 1;
		while (1) {
			fd_set rfds;
			FD_ZERO(&rfds);
			if (!nopm)
				FD_SET(pty, &rfds);
			FD_SET(sck, &rfds);
			if (fake > 0)
				FD_SET(fake, &rfds);
			select(maxfd, &rfds, NULL, NULL, NULL);
			if (FD_ISSET(fake, &rfds))
				handleFakePacket(fake, pty);
			if (FD_ISSET(sck, &rfds))
				handlePhonetPacket(sck, pty);
			if (FD_ISSET(pty, &rfds)) {
				char buf[BUFFER_SIZE];
				if (read(pty, buf, sizeof(buf)) < 1)
					// EOF or error, idle GPS
					break;
			}
		}
		// 4. Until EOF from master fd
		debug("TTY idle, shutting off GPS\n");
		cellmoStopGPS(sck);
		isActive = 0;
		// 5. Reopen slave fd, and start over
	}
}
